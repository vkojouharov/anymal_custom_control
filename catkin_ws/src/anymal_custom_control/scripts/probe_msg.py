#!/usr/bin/env python3
"""Probe a ROS topic publisher to extract the real message definition.

Connects via TCPROS with md5sum=* (wildcard) to get the full definition
from the publisher's connection header, even without local message classes.
"""
import socket
import struct
import xmlrpc.client

import rospy
import rosgraph


def probe_topic(topic):
    master = rosgraph.Master('/msg_probe')

    # Get topic type
    topic_type = None
    for t, typ in master.getTopicTypes():
        if t == topic:
            topic_type = typ
            break

    if not topic_type:
        print(f"Topic {topic} not found")
        return

    # Find a publisher node
    pub_nodes = []
    for t, nodes in master.getSystemState()[0]:
        if t == topic:
            pub_nodes = nodes
            break

    if not pub_nodes:
        print(f"No publishers for {topic}")
        return

    print(f"Topic type: {topic_type}")
    print(f"Publishers: {pub_nodes}")

    # Try each publisher
    for node_name in pub_nodes:
        print(f"\nProbing publisher: {node_name}")
        try:
            node_uri = master.lookupNode(node_name)
            proxy = xmlrpc.client.ServerProxy(node_uri)
            code, msg, protos = proxy.requestTopic(
                '/msg_probe', topic, [['TCPROS']])

            if code != 1:
                print(f"  requestTopic failed: {msg}")
                continue

            _, host, port = protos
            print(f"  Connecting to {host}:{port}")

            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(5.0)
            sock.connect((host, port))

            # Send subscriber header with wildcard md5sum
            fields = [
                f'callerid=/msg_probe',
                f'topic={topic}',
                f'md5sum=*',
                f'type={topic_type}',
            ]
            header_data = b''
            for f in fields:
                fb = f.encode('utf-8')
                header_data += struct.pack('<I', len(fb)) + fb
            sock.sendall(struct.pack('<I', len(header_data)) + header_data)

            # Read response header
            buf = _recv_exact(sock, 4)
            header_len = struct.unpack('<I', buf)[0]
            buf = _recv_exact(sock, header_len)

            # Parse response fields
            pos = 0
            resp = {}
            while pos < len(buf):
                fl = struct.unpack('<I', buf[pos:pos + 4])[0]
                pos += 4
                field = buf[pos:pos + fl].decode('utf-8', errors='replace')
                pos += fl
                if '=' in field:
                    k, v = field.split('=', 1)
                    resp[k] = v

            sock.close()

            print(f"\n  md5sum: {resp.get('md5sum', 'N/A')}")
            print(f"  type: {resp.get('type', 'N/A')}")
            print(f"\n=== message_definition ===")
            print(resp.get('message_definition', 'N/A'))
            print("=== end ===")
            return resp

        except Exception as e:
            print(f"  Error: {e}")
            continue


def _recv_exact(sock, n):
    buf = b''
    while len(buf) < n:
        chunk = sock.recv(n - len(buf))
        if not chunk:
            raise ConnectionError("Connection closed")
        buf += chunk
    return buf


if __name__ == '__main__':
    import sys
    rospy.init_node('msg_probe', anonymous=True)
    topic = sys.argv[1] if len(sys.argv) > 1 else \
        '/operational_mode_manager/switch_operational_mode/goal'
    probe_topic(topic)
