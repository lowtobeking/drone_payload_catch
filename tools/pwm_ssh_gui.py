#!/usr/bin/env python3
"""Desktop PWM33 bench control over strict-host-key OpenSSH. Standard library only."""
import argparse
import base64
import importlib.util
import json
import logging
from logging.handlers import RotatingFileHandler
import os
from pathlib import Path
import queue
import re
import shlex
import shutil
import subprocess
import sys
import threading
import time
import tkinter as tk
from tkinter import ttk, messagebox
import zlib

MIN_US, MAX_US = 1200, 1800

def pulse_value(value):
    text=str(value).strip()
    if not re.fullmatch(r'[0-9]+',text):
        raise ValueError('脉宽必须为整数微秒')
    pulse=int(text)
    lower,upper=MIN_US,MAX_US
    if not lower<=pulse<=upper:
        raise ValueError('当前允许脉宽：%d–%dµs'%(lower,upper))
    return pulse


AGENT = Path(__file__).with_name('pwm_remote_agent.py')


def diagnostic_logger():
    logger=logging.getLogger('pwm33.'+str(os.getpid()))
    logger.setLevel(logging.INFO)
    if logger.handlers:return logger, getattr(logger,'log_path',None)
    folder=(Path(os.environ.get('LOCALAPPDATA',str(Path.home()/'AppData/Local')))/'PWM33/logs'
            if os.name=='nt' else Path.home()/'.local/state/pwm33')
    try:
        folder.mkdir(parents=True,exist_ok=True)
        path=folder/('pwm33-'+str(os.getpid())+'.log')
        handler=RotatingFileHandler(path,maxBytes=1_000_000,backupCount=2,encoding='utf-8')
        handler.setFormatter(logging.Formatter('%(asctime)s %(message)s'))
        logger.addHandler(handler);logger.log_path=path
        return logger,path
    except OSError:
        logger.addHandler(logging.NullHandler())
        return logger,None


def ssh_command(host, user, key, alias):
    for text in (host, user, alias):
        if not re.fullmatch(r'[A-Za-z0-9_.:-]+', text) or text.startswith('-'):
            raise ValueError('主机、用户或指纹别名格式不正确')
    key = Path(key).expanduser()
    if not key.is_file():
        raise ValueError('找不到 SSH 私钥文件')
    ssh = shutil.which('ssh')
    if not ssh:
        raise ValueError('找不到 OpenSSH 客户端')
    source = base64.b64encode(zlib.compress(AGENT.read_bytes())).decode('ascii')
    bootstrap = "exec(__import__('zlib').decompress(__import__('base64').b64decode('"+source+"')))"
    remote = 'sudo -n /usr/bin/python3 -u -c '+shlex.quote(bootstrap)
    return [ssh, '-T', '-o', 'BatchMode=yes', '-o', 'StrictHostKeyChecking=yes',
            '-o', 'ConnectTimeout=8', '-o', 'ServerAliveInterval=5', '-o', 'ServerAliveCountMax=3',
            '-o', 'HostKeyAlias='+alias, '-i', str(key), user+'@'+host, remote]


class Connection:
    def __init__(self, command):
        self.events = queue.Queue()
        options = dict(stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                       text=True, encoding='utf-8', errors='replace', bufsize=1)
        if os.name == 'nt':
            options['creationflags'] = subprocess.CREATE_NO_WINDOW
        self.process = subprocess.Popen(command, **options)
        for stream, kind in ((self.process.stdout, 'message'), (self.process.stderr, 'log')):
            threading.Thread(target=self._read, args=(stream,kind), daemon=True).start()

    def _read(self, stream, kind):
        try:
            for line in stream:
                self.events.put((kind,line.strip()))
        finally:
            self.events.put(('eof',kind))

    def send(self, request):
        if self.process.poll() is not None:
            raise RuntimeError('SSH 连接已退出')
        self.process.stdin.write(json.dumps(request,allow_nan=False)+'\n')
        self.process.stdin.flush()

    def alive(self):
        return self.process.poll() is None

    def close(self):
        # EOF triggers remote finally; remote watchdog also stops after 2s without UI ping.
        try:
            self.process.stdin.close()
        except (OSError, ValueError):
            pass
        def cleanup():
            try:
                self.process.wait(timeout=4)
            except subprocess.TimeoutExpired:
                self.process.terminate()
                try:self.process.wait(timeout=2)
                except subprocess.TimeoutExpired:self.process.kill()
            for stream in (self.process.stdout,self.process.stderr):
                try:stream.close()
                except OSError:pass
        threading.Thread(target=cleanup,daemon=True).start()


class DemoConnection:
    def __init__(self):
        spec = importlib.util.spec_from_file_location('pwm_demo_agent', AGENT)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        self.controller = module.Controller(module.MockBackend(),time.monotonic())
        self.events = queue.Queue()
        self.open = True

    def poll(self):
        if not self.open:return
        self.controller.tick(time.monotonic())
        self.events.put(('message',json.dumps(self.controller.status(time.monotonic()))))
        if self.controller.expired:self.open=False

    def send(self, request):
        self.controller.command(request,time.monotonic())
        self.events.put(('message',json.dumps(self.controller.status(time.monotonic()))))

    def alive(self):return self.open
    def close(self):
        self.controller.stop()
        self.open=False


class App:
    def __init__(self, root, demo=False):
        self.root,self.demo = root,demo
        self.logger,self.log_path=diagnostic_logger()
        self.last_reason=None
        self.remote_error=''
        self.connection = None
        self.seq = 0
        self.status = None
        self.last_status = 0
        self.last_ping = 0
        self.closing = False
        self.stale_reported = False
        root.title('Jetson 33脚 · 舵机 PWM 控制'+('【模拟】' if demo else ''))
        root.geometry('860x800')
        root.minsize(820,770)
        style=ttk.Style()
        if 'clam' in style.theme_names():style.theme_use('clam')
        style.configure('Title.TLabel',font=('Microsoft YaHei UI',18,'bold'))
        style.configure('Stop.TButton',font=('Microsoft YaHei UI',13,'bold'),foreground='#b42318')
        main=ttk.Frame(root,padding=20);main.pack(fill='both',expand=True)
        ttk.Label(main,text='物理 33 脚 · 舵机调试',style='Title.TLabel').pack(anchor='w')
        ttk.Label(main,text='EMAX ES08MD II  ·  外部 5V / 共地  ·  50Hz 地面调试起点（待实测）').pack(anchor='w',pady=(3,12))
        network=ttk.LabelFrame(main,text='SSH 连接',padding=10);network.pack(fill='x')
        self.host=tk.StringVar(value='192.168.155.186')
        self.user=tk.StringVar(value='nvidia')
        self.key=tk.StringVar(value=str(Path.home()/'.ssh/id_ed25519'))
        self.alias=tk.StringVar(value='192.168.155.186')
        for row,(label,var) in enumerate((('Jetson 地址',self.host),('用户',self.user),('私钥路径',self.key),('已保存指纹别名',self.alias))):
            ttk.Label(network,text=label).grid(row=row,column=0,sticky='w',pady=2)
            ttk.Entry(network,textvariable=var).grid(row=row,column=1,sticky='ew',padx=10,pady=2)
        network.columnconfigure(1,weight=1)
        buttons=ttk.Frame(network);buttons.grid(row=4,column=0,columnspan=2,sticky='w',pady=(6,0))
        self.connect_button=ttk.Button(buttons,text='连接（不输出脉冲）',command=self.connect);self.connect_button.pack(side='left')
        ttk.Button(buttons,text='断开连接',command=self.disconnect).pack(side='left',padx=8)
        self.connection_text=tk.StringVar(value='未连接'+(' · 模拟模式，不接触硬件' if demo else ''))
        ttk.Label(main,textvariable=self.connection_text,wraplength=750).pack(anchor='w',pady=(10,2))
        self.hardware_text=tk.StringVar(value='等待设备状态')
        ttk.Label(main,textvariable=self.hardware_text,font=('Microsoft YaHei UI',11,'bold'),wraplength=800).pack(anchor='w',pady=(0,8))
        self.confirm=tk.BooleanVar(value=False)
        ttk.Checkbutton(main,text='已确认无桨、地面调试，舵机可自由转动且不会误释放载荷',variable=self.confirm).pack(anchor='w')
        controls=ttk.LabelFrame(main,text='输出设置',padding=12);controls.pack(fill='x',pady=10)
        self.pulse=tk.StringVar(value='1500')
        row=ttk.Frame(controls);row.pack(fill='x')
        ttk.Label(row,text='目标高电平脉宽（µs）').pack(side='left')
        self.pulse_input=ttk.Spinbox(row,from_=MIN_US,to=MAX_US,increment=10,textvariable=self.pulse,width=9)
        self.pulse_input.pack(side='left',padx=8)
        self.conversion_text=tk.StringVar()
        ttk.Label(controls,textvariable=self.conversion_text).pack(anchor='w',pady=(7,0))
        ttk.Label(controls,text='持续保持，无输出倒计时；修改脉宽后点击“应用脉宽 / 保持”。').pack(anchor='w',pady=(5,0))
        self.pulse.trace_add('write',lambda *args:self.update_conversion())
        shortcuts=ttk.Frame(controls);shortcuts.pack(anchor='w')
        for value in (1400,1500,1600):
            ttk.Button(shortcuts,text='%dµs'%value,width=10,command=lambda v=value:self.pulse.set(str(v))).pack(side='left',padx=(0,6))
        ttk.Label(controls,text='预设只填写数值，点击应用才动作。直接输出目标脉宽，不缓变、不先回中。',wraplength=770).pack(anchor='w',pady=(7,0))
        ttk.Label(controls,text='50Hz下硬件脉宽约78µs一档；显示和回读为软件请求值，实际引脚波形尚未测量。',wraplength=770).pack(anchor='w',pady=(5,0))
        self.update_conversion()
        actions=ttk.Frame(main);actions.pack(fill='x',pady=(0,8))
        ttk.Button(actions,text='开启 PWM / 配置 50Hz',command=lambda:self.action('prepare')).pack(side='left')
        ttk.Button(actions,text='应用脉宽 / 保持',command=self.start).pack(side='left',padx=8)
        ttk.Button(actions,text='停止输出',style='Stop.TButton',command=self.stop).pack(side='left')
        ttk.Button(actions,text='恢复 GPIO',command=self.restore).pack(side='right')
        ttk.Label(main,text='停止/断线：只停脉冲，保留 PWM 模式；停止信号不等于锁扣已锁住。',wraplength=760).pack(anchor='w')
        self.log_box=tk.Text(main,height=7,wrap='word',font=('Microsoft YaHei UI',9),state='disabled')
        self.log_box.pack(fill='both',expand=True,pady=(10,0))
        root.protocol('WM_DELETE_WINDOW',self.close)
        root.bind('<Escape>',lambda e:self.stop())
        self.log('准备就绪。直接输入脉宽，默认1500µs，允许1200–1800µs；无输出时长。连接不会发脉冲。')
        self.log('日志：'+str(self.log_path) if self.log_path else '本地诊断日志不可用')
        root.after(100,self.poll)

    def log(self,text):
        self.logger.info(text)
        self.log_box.configure(state='normal')
        self.log_box.insert('end',time.strftime('%H:%M:%S')+'  '+text+'\n')
        if int(self.log_box.index('end-1c').split('.')[0])>160:self.log_box.delete('1.0','40.0')
        self.log_box.see('end');self.log_box.configure(state='disabled')

    def connect(self):
        if self.connection:return
        try:
            self.connection=DemoConnection() if self.demo else Connection(ssh_command(
                self.host.get().strip(),self.user.get().strip(),self.key.get().strip(),self.alias.get().strip()))
            self.status=None;self.seq=0;self.last_ping=0;self.stale_reported=False
            self.last_reason=None;self.remote_error=''
            self.connection_text.set('正在连接…')
            self.connect_button.configure(state='disabled')
            self.log('连接仅查询状态；不会自动准备引脚或输出。')
        except Exception as exc:messagebox.showerror('连接失败',str(exc))

    def send(self,action,**values):
        if not self.connection or not self.status:raise RuntimeError('尚未收到设备状态')
        # Never let missing telemetry suppress the heartbeat itself. The server
        # still validates its challenge and independently stops output after 2s.
        if action in ('set','prepare','restore') and self.status.get('control_ready') is False:
            raise RuntimeError(self.status.get('control_block_reason','当前只读连接，禁止操作PWM'))
        if action not in ('stop','close','ping') and time.monotonic()-self.last_status>1.2:
            raise RuntimeError('状态已过期；禁止输出，请重连')
        self.seq+=1
        self.connection.send(dict(action=action,seq=self.seq,session=self.status['session'],
                                  token=self.status['token'],**values))

    def action(self,action):
        if action in ('prepare','restore') and not self.confirm.get():
            messagebox.showwarning('先确认现场条件','请勾选地面调试确认。');return
        try:self.send(action);self.log('已发送：'+dict(prepare='开启模式',restore='恢复GPIO')[action])
        except Exception as exc:messagebox.showerror('操作失败',str(exc))

    def update_conversion(self):
        lower,upper=MIN_US,MAX_US
        self.pulse_input.configure(from_=lower,to=upper)
        try:
            pulse=pulse_value(self.pulse.get())
            self.conversion_text.set('请求 %dµs / 周期20ms（50Hz）/ 占空比%.2f%% · 允许%d–%dµs' %
                                     (pulse,pulse/200,lower,upper))
        except ValueError:
            self.conversion_text.set('请输入%d–%dµs的整数；超限禁止输出。'%(lower,upper))

    def start(self):
        if not self.confirm.get():
            messagebox.showwarning('先确认现场条件','请勾选地面调试确认。');return
        try:
            pulse=pulse_value(self.pulse.get())
            self.send('set',pulse_us=pulse,seconds=0)
            self.log('请求目标%dµs，持续保持；等待设备确认。'%pulse)
        except Exception as exc:messagebox.showerror('不能应用脉宽',str(exc))

    def stop(self):
        if not self.connection:return
        try:self.send('stop');self.log('已请求停止脉冲，保留PWM模式。')
        except Exception as exc:
            self.log('停止未获确认：'+str(exc)+'。断开后远端看门狗会尝试停止；必要时断开舵机电源。')
            self.disconnect()

    def restore(self):
        if messagebox.askyesno('恢复引脚','停止脉冲并将33脚恢复初始GPIO状态？'):
            self.action('restore')

    def disconnect(self):
        connection=self.connection
        if connection:
            try:self.send('close')
            except Exception:pass
            connection.close()
        self.connection=None;self.status=None
        self.connect_button.configure(state='normal')
        self.connection_text.set('已断开；未取得最终回读时，输出状态需重新连接核实')
        self.hardware_text.set('状态未知 · 远端心跳超时约2秒后尝试停止脉冲')

    def poll(self):
        connection=self.connection
        if connection:
            if isinstance(connection,DemoConnection):connection.poll()
            for _ in range(80):
                try:kind,line=connection.events.get_nowait()
                except queue.Empty:break
                if kind=='message':
                    try:d=json.loads(line)
                    except ValueError:
                        self.log('远端：'+line[-300:]);continue
                    if d.get('type')=='status':
                        if self.status and d['session']!=self.status['session']:
                            self.log('远端会话改变；停止并重连。');self.disconnect();break
                        self.status=d;self.last_status=time.monotonic();self.stale_reported=False
                        if d.get('reason')!=self.last_reason:
                            self.last_reason=d.get('reason')
                            self.log('远端状态：'+str(self.last_reason))
                        self.connection_text.set(('模拟连接' if self.demo else 'SSH 已连接')+' · 33脚 · '+d.get('reason',''))
                        if d['enabled']:
                            pulse=d['pulse_us'];ratio=pulse*1000/d['period_ns']*100
                            target=d.get('target_us')
                            target_text=('%sµs'%target) if target is not None else '未知'
                            self.hardware_text.set('脉宽请求回读：%.0fµs / %.1fHz / %.2f%% → 目标%s；非线端实测' %
                                                   (pulse,1e9/d['period_ns'],ratio,target_text))
                        else:self.hardware_text.set('无脉冲输出 · '+('PWM 模式保持' if d['mode'] else '初始 GPIO 模式'))
                        if d.get('control_ready') is False:
                            self.connection_text.set('SSH 已连接 · 仅可查看（安全基线过期）')
                            self.hardware_text.set('无脉冲输出 · 初始GPIO · '+d.get('control_block_reason','禁止PWM操作'))
                        if d.get('heartbeat_lost'):
                            self.hardware_text.set('心跳中断：脉冲已停止，连接等待恢复；恢复后需手动应用脉宽。')
                        if d['mode'] and not d.get('profile_ready',False):
                            self.hardware_text.set(self.hardware_text.get()+' · 旧频率，先停止再配置50Hz')
                    elif d.get('type')=='error':
                        self.remote_error=d.get('error','')
                        self.log('远端错误：'+self.remote_error+('（输出已停止，连接保留）' if d.get('recoverable') else ''))
                elif kind=='log' and line:self.log(line[-500:])
            if self.connection and self.status and time.monotonic()-self.last_status>1.2:
                self.hardware_text.set('状态过期 · 输出状态未知，禁止新动作')
                if not self.stale_reported:self.log('收不到新鲜状态；请停止或断开，并检查网络。');self.stale_reported=True
            if self.connection and self.status and time.monotonic()-self.last_ping>=.4:
                try:self.send('ping');self.last_ping=time.monotonic()
                except Exception:pass
            if self.connection and not connection.alive():
                code=None if isinstance(connection,DemoConnection) else connection.process.poll()
                self.log('SSH/模拟会话已退出，退出码='+str(code)+'，最后远端状态='+str(self.last_reason)+
                         ('，错误='+self.remote_error if self.remote_error else ''))
                self.disconnect()
        if not self.closing:self.root.after(100,self.poll)

    def close(self):
        self.closing=True;self.disconnect();self.root.destroy()


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--demo',action='store_true',help='simulate locally; never SSH or drive hardware')
    parser.add_argument('--smoke-test',action='store_true',help='open demo UI briefly, then close')
    args=parser.parse_args()
    root=tk.Tk();app=App(root,demo=args.demo or args.smoke_test)
    smoke_errors=[]
    if args.smoke_test:
        # Fail visibly on callback errors; a closed window alone is not a test.
        def failed(kind,error,traceback):
            smoke_errors.append(str(error))
            app.close()
        root.report_callback_exception=failed
        def popup_error(title,text,*args,**kwargs):
            raise AssertionError(title+': '+text)
        messagebox.showerror=messagebox.showwarning=popup_error
        def exercise():
            app.connect();app.confirm.set(True)
            assert not hasattr(app,'duration') and not hasattr(app,'continuous') and not hasattr(app,'angle')
        def verify_hold():
            control=app.connection.controller
            assert control.running and control.target==1500 and control.deadline is None
            assert app.status['period_ns']==20_000_000
            stamp=app.last_status
            app.last_status=time.monotonic()-2
            app.send('ping')  # Stale status must never suppress heartbeat recovery.
            app.last_status=stamp
            assert not hasattr(app,'expanded')
            assert float(app.pulse_input.cget('from'))==1200
            assert float(app.pulse_input.cget('to'))==1800
            for boundary in (1200,1800,1700):
                assert pulse_value(str(boundary))==boundary
                app.pulse.set(str(boundary));app.start()
                assert control.target==boundary and control.deadline is None
                assert control.backend.pulse==boundary and control.current==boundary
            app.pulse.set('1533');app.start()
            assert control.target==1533 and control.current == 1533
            assert control.deadline is None
            for invalid in ('nan','inf','1500.5','1199','1801','999','2001','bad',''):
                try:pulse_value(invalid)
                except ValueError:pass
                else:raise AssertionError('invalid pulse accepted: '+invalid)
        def verify_stop():
            assert not app.connection.controller.running
            assert not app.status['enabled'] and app.status['mode']
        root.after(100,exercise)
        root.after(800,lambda:app.action('prepare'))
        root.after(1300,app.start)
        root.after(1800,verify_hold)
        root.after(2500,app.stop)
        root.after(2800,verify_stop)
        root.after(3000,app.close)
    root.mainloop()
    if args.smoke_test:
        if smoke_errors:raise RuntimeError('GUI smoke failed: '+'; '.join(smoke_errors))
        print('GUI smoke passed: immediate direct 1700/1533us commands, no angle mapping or duration, continuous hold, 1200..1800us limits and input rejection, heartbeat, stop retaining PWM mode; no SSH.')


if __name__=='__main__':main()
