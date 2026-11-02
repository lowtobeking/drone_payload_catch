"""Offline network-helper tests: mock kernel routes, no real networking."""
import importlib.util
import ipaddress
from pathlib import Path
import socket
import struct
import sys
import tempfile
import unittest
from unittest.mock import patch
spec=importlib.util.spec_from_file_location("peer",Path(__file__).with_name("wifi_peer_route.py"))
peer=importlib.util.module_from_spec(spec)
spec.loader.exec_module(peer)
class Tests(unittest.TestCase):
    def setUp(self):
        self.rows=[]
        self.commands=[]
        self.tmp=tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.state=patch.object(peer,"STATE",Path(self.tmp.name)/"state.json")
        self.state.start()
        self.addCleanup(self.state.stop)
        self.p=patch.object(peer,"route_rows",lambda:list(self.rows))
        self.p.start()
        self.addCleanup(self.p.stop)
        def command(args):
            self.commands.append(args)
            if args[3]=="add":
                self.rows.append(dict(dst=args[4],dev=peer.IFACE,protocol=186,metric=9))
            elif args[3]=="del":
                self.rows[:]=[r for r in self.rows if r["dst"]!=args[4]]
            return ""
        self.cmd=patch.object(peer,"command",command)
        self.cmd.start()
        self.addCleanup(self.cmd.stop)
        self.manager=peer.RouteManager()
    def test_only_authorized_mac(self):
        self.assertFalse(self.manager.observe(bytes(6),"192.168.1.4","192.168.1.5/22",1))
        self.assertEqual(self.commands,[])
    def test_dynamic_ip_changes_remove_only_owned(self):
        self.rows=[dict(dst="192.168.1.111",dev=peer.IFACE,protocol="static",metric=10)]
        self.assertTrue(self.manager.observe(peer.LAPTOP_MAC,"192.168.1.4","192.168.1.5/22",1))
        self.assertTrue(self.manager.observe(peer.LAPTOP_MAC,"192.168.1.9","192.168.1.5/22",2))
        self.assertTrue(any(r["dst"]=="192.168.1.111" for r in self.rows))
        self.assertFalse(any(r["dst"]=="192.168.1.4/32" for r in self.rows))
    def test_reserved_outside_and_own_ip_rejected(self):
        for ip in ["192.168.1.101","192.168.1.203","192.168.1.111","192.168.1.123","10.41.10.2","192.168.1.5","192.168.3.255"]:
            self.assertFalse(self.manager.observe(peer.LAPTOP_MAC,ip,"192.168.1.5/22",1))
        self.assertFalse(self.commands)
    def test_lease_expiry_removes_route(self):
        self.manager.observe(peer.LAPTOP_MAC,"192.168.1.4","192.168.1.5/22",1)
        self.manager.expire(122)
        self.assertFalse(self.rows)
    def test_conflicting_mac_removes_route(self):
        self.manager.observe(peer.LAPTOP_MAC,"192.168.1.4","192.168.1.5/22",1)
        self.manager.observe(bytes(6),"192.168.1.4","192.168.1.5/22",2)
        self.assertFalse(self.rows)
    def test_existing_host_route_on_mesh_refused(self):
        self.rows=[dict(dst="192.168.1.4",dev="enx00e04c680065",protocol="static",metric=1)]
        self.assertFalse(self.manager.observe(peer.LAPTOP_MAC,"192.168.1.4","192.168.1.5/22",1))
        self.assertFalse(self.commands)
    def test_arp_parser_requires_ethernet_source_match(self):
        frame=(bytes(6)+peer.LAPTOP_MAC+b"\x08\x06"+struct.pack("!HHBBH",1,0x800,6,4,1)
               +peer.LAPTOP_MAC+socket.inet_aton("192.168.1.4")+bytes(10))
        self.assertEqual(peer.parse_arp(frame),(peer.LAPTOP_MAC,"192.168.1.4"))
        self.assertIsNone(peer.parse_arp(frame[:6]+bytes(6)+frame[12:]))
        self.assertIsNone(peer.parse_arp(frame[:20]))
    def test_named_protocol_is_owned(self):
        self.assertTrue(peer.is_owned(dict(dst='192.168.1.4',dev=peer.IFACE,protocol='bgp',metric=9)))
    def test_cleanup_preserves_other_interfaces(self):
        self.rows=[dict(dst="10.41.10.0/24",dev="eno1",protocol="kernel",metric=101),
                   dict(dst="192.168.1.0/24",dev="enx00e04c680065",protocol="kernel",metric=100)]
        self.manager.clear()
        self.assertEqual(len(self.rows),2)
        self.assertFalse(self.commands)
if __name__=="__main__":
    unittest.main()
