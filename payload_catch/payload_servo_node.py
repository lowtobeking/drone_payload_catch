"""A-aircraft latch adapter. Raw release_at never reaches PWM."""
import json
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
from .payload_servo import ServoConfig, ServoInterlock, AttemptRecord, servo_signature
from .servo_pwm import PWMWorker, MockPWM


class PayloadServo(Node):
    def __init__(self):
        super().__init__('payload_servo')
        self.declare_parameter('release_id','')
        self.release_id=self.get_parameter('release_id').value
        defaults = dict(enabled=False, calibrated=False, confirmation_mode='sensor', backend='jetson', neutral_us=1500,
                        open_us=1700, period_ns=20000000, release_delay_s=1.5,
                        delay_reference='local_command_direct', max_output_s=2.0,
                        delay_tolerance_s=.02, feedback_timeout_s=.15, owner=0,
                        attempt_record='/var/lib/payload_catch/pwm33_attempt.json')
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        values = {k: self.get_parameter(k).value for k in defaults}
        config = ServoConfig(**{k: values[k] for k in ServoConfig.__dataclass_fields__})
        config.validate()
        mock = values['backend'] == 'mock'
        if values['backend'] not in ('jetson', 'mock'):
            raise ValueError('backend must be jetson or mock')
        # Mock observations use separate topics; cannot satisfy real flight prechecks.
        if not mock and self.get_parameter('use_sim_time').value:
            raise ValueError('real PWM requires synchronized wall clock, not use_sim_time')
        prefix = '/payload/mock' if mock else '/payload'
        self.ready_pub = self.create_publisher(Bool, prefix + '/hardware_ready', 10)
        self.status_pub = self.create_publisher(String, prefix + '/hardware_status', 10)
        self.released_pub = self.create_publisher(Bool, prefix + '/released', 10)
        self.release_event_pub = self.create_publisher(String, prefix + '/release_event', 10)
        self.action_complete_pub = self.create_publisher(String, prefix + '/release_action_complete', 10)
        self.action_reported=False
        self.abort_pub = self.create_publisher(Bool, '/payload/release_abort', 10)
        self.request_pub = self.create_publisher(String, '/safety/request', 10)
        self.backend = MockPWM()
        self.core = ServoInterlock(config, self.backend, AttemptRecord(values['attempt_record']), mock=mock)
        if not mock and config.enabled and config.calibrated and self.core.state != 'FAULT':
            try:
                self.backend = PWMWorker(config.period_ns, max_output_s=config.max_output_s)
                self.core.backend = self.backend
            except (RuntimeError, OSError) as exc:
                self.core.fault(str(exc))
        self.fault_reported = False
        self.released_reported = False
        self.output_closed = False
        self.create_subscription(String, '/safety/state', self._supervisor, 10)
        self.create_subscription(String, prefix + '/latch_feedback', self._feedback, 10)
        self.create_subscription(String, '/payload/authorized_release', self._authorized, 10)
        self.create_subscription(Bool, '/payload/release_abort', self._abort, 10)
        for action in ('pause', 'land', 'kill'):
            for suffix in ('', '_a', '_b'):
                self.create_subscription(Bool, '/safety/' + action + suffix, self._abort, 10)
        self.timer = self.create_timer(.02, self._tick)

    def _data(self, message):
        data = json.loads(message.data)
        if not isinstance(data, dict):
            raise ValueError('object expected')
        return data

    def _supervisor(self, message):
        try:
            self.core.on_supervisor(self._data(message), time.time(), time.monotonic())
        except (ValueError, TypeError):
            self.core.supervisor_watch.valid = False
        self._sync()

    def _feedback(self, message):
        try:
            self.core.on_feedback(self._data(message), time.time(), time.monotonic())
        except (ValueError, TypeError):
            self.core.feedback_watch.valid = False
        self._sync()

    def _authorized(self, message):
        try:
            accepted=self.core.authorize(self._data(message), time.time(), time.monotonic())
            if (not accepted and self.core.command_wall is None and self.core.config.enabled
                    and self.core.config.calibrated and self.core.state!='FAULT'):
                self.core.fault('authorized release rejected: readiness / session / timing interlock')
        except (ValueError, TypeError):
            self.core.fault('malformed authorization')
        self._sync()

    def _abort(self, message):
        if message.data:
            self.core.abort()
            self._sync()

    def _close_output(self):
        if self.output_closed:
            return
        self.output_closed = True
        try:
            self.backend.close()
        except (RuntimeError, OSError) as exc:
            reason='PWM rollback needs inspection: '+str(exc)
            if self.core.reason:reason=self.core.reason+'; '+reason
            self.core.fault(reason)
            self.core.reason=reason
            self.get_logger().error(reason)

    def _sync(self):
        if self.core.state=='ACTION_COMPLETE' and not self.action_reported:
            # Report completion only after the PWM worker confirms stop/rollback.
            self._close_output()
            if self.core.state!='FAULT':
                event=dict(stamp=time.time(),session=self.core.session,nonce=self.core.authorization_nonce,
                    confirmation_mode='timed',physically_confirmed=False,detached_at=None,
                    action_completed_at=self.core.action_completed_at,estimated_detach_at=self.core.estimated_detach_at,
                    planned_at=self.core.planned_at,command_at=self.core.scheduled_command_at,
                    output_written_at=self.core.output_written_at,mock=self.core.mock,release_id=self.release_id,
                    servo_signature=servo_signature(self.core.config))
                message=String(data=json.dumps(event,allow_nan=False))
                self.release_event_pub.publish(message);self.action_complete_pub.publish(message)
                self.action_reported=True
                self.get_logger().warn('Timed latch action complete; physical detachment UNCONFIRMED')
        if self.core.physical_released and not self.released_reported:
            # Preserve the SENSOR edge through DDS latency; publish before rollback.
            self.release_event_pub.publish(String(data=json.dumps(dict(
                stamp=time.time(),session=self.core.session,nonce=self.core.authorization_nonce,
                detached_at=self.core.detached_at,planned_at=self.core.planned_at,
                command_at=self.core.scheduled_command_at,output_written_at=self.core.output_written_at,
                mock=self.core.mock,confirmation_mode='sensor',physically_confirmed=True,release_id=self.release_id,
                servo_signature=servo_signature(self.core.config)),allow_nan=False)))
            self.released_pub.publish(Bool(data=True))
            self.released_reported = True
            # Continuous rotation stops immediately after the physical edge.
            self._close_output()
        if self.core.state == 'FAULT':
            self._close_output()
            if not self.fault_reported:
                self.fault_reported = True
                self.get_logger().error('LATCH FAULT: ' + self.core.reason)
                if self.core.config.enabled and not self.core.mock:
                    self.abort_pub.publish(Bool(data=True))
                    for drone_id in (0, 1):
                        self.request_pub.publish(String(data=json.dumps(dict(
                            stamp=time.time(), drone_id=drone_id, action='pause', reason=self.core.reason))))

    def _tick(self):
        wall, mono = time.time(), time.monotonic()
        self.core.tick(wall, mono)
        self._sync()
        if not self.output_closed and self.core.config.enabled and self.core.config.calibrated:
            try:
                self.backend.heartbeat()
            except (RuntimeError, OSError) as exc:
                self.core.fault(str(exc))
                self._sync()
        ready = self.core.hardware_ready(wall, mono)
        self.ready_pub.publish(Bool(data=ready))
        self.status_pub.publish(String(data=json.dumps(dict(
            stamp=wall, session=self.core.session, ready=ready, state=self.core.state,
            release_id=self.release_id,servo_signature=servo_signature(self.core.config),
            reason=self.core.reason, mock=self.core.mock,
            feedback_stamp=self.core.feedback_watch.stamp,confirmation_mode=self.core.config.confirmation_mode,
            feedback_required=self.core.config.confirmation_mode=='sensor',
            action_completed_at=self.core.action_completed_at,estimated_detach_at=self.core.estimated_detach_at,
            command_sent_at=self.core.command_wall, planned_detach_at=self.core.planned_at,
            scheduled_command_at=self.core.scheduled_command_at,
            output_written_at=self.core.output_written_at,detached_at=self.core.detached_at,
            measured_opening_delay_s=(None if self.core.output_written_at is None or self.core.detached_at is None
                                      else self.core.detached_at-self.core.output_written_at),
            release_timing_error_s=(None if self.core.planned_at is None or self.core.detached_at is None
                                    else self.core.detached_at-self.core.planned_at),
            release_delay_s=self.core.config.release_delay_s,
            delay_reference=self.core.config.delay_reference,
            opening_elapsed_s=(None if self.core.command_mono is None else max(0,mono-self.core.command_mono)),
            physically_released=self.core.physical_released))))

    def destroy_node(self):
        self._close_output()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = PayloadServo()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
