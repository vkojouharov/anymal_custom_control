"""Low-level Dynamixel SDK wrapper.

The functional API in ``driver.py`` is usually what you want.
Use ``DynamixelController`` directly only when you need ad-hoc register
reads/writes, custom sync groups, or motors not covered by the functional API.
"""
from dynamixel_sdk import (
    COMM_SUCCESS,
    GroupSyncRead,
    GroupSyncWrite,
    PacketHandler,
    PortHandler,
)


class DynamixelController:
    """Thin wrapper over PortHandler + PacketHandler.

    Attributes:
        port_handler: SDK PortHandler — exposed for GroupSync* consumers.
        packet_handler: SDK PacketHandler — exposed for GroupSync* consumers.
    """

    def __init__(self, device_name, baudrate, protocol_version=2.0):
        self.device_name = device_name
        self.baudrate = baudrate
        self.protocol_version = protocol_version

        self.port_handler = PortHandler(device_name)
        self.packet_handler = PacketHandler(protocol_version)

        if not self.port_handler.openPort():
            raise RuntimeError(f"Failed to open port {device_name}")
        if not self.port_handler.setBaudRate(baudrate):
            self.port_handler.closePort()
            raise RuntimeError(f"Failed to set baudrate {baudrate} on {device_name}")

    # ── register IO ─────────────────────────────────────────────────────────

    def WRITE(self, dxl_id, command_type, value):
        """Write ``value`` to (address, length) tuple ``command_type``.

        Returns True on success, False on comm/packet error (and prints).
        """
        address, length = command_type
        if length == 1:
            comm, err = self.packet_handler.write1ByteTxRx(self.port_handler, dxl_id, address, value)
        elif length == 2:
            comm, err = self.packet_handler.write2ByteTxRx(self.port_handler, dxl_id, address, value)
        elif length == 4:
            comm, err = self.packet_handler.write4ByteTxRx(self.port_handler, dxl_id, address, value)
        else:
            print(f"[dxl] Invalid byte length: {length}")
            return False
        return self._check(dxl_id, comm, err, "WRITE")

    def READ(self, dxl_id, command_type):
        """Read value at (address, length) tuple ``command_type``.

        Returns the int value on success, or None on error.
        """
        address, length = command_type
        if length == 1:
            value, comm, err = self.packet_handler.read1ByteTxRx(self.port_handler, dxl_id, address)
        elif length == 2:
            value, comm, err = self.packet_handler.read2ByteTxRx(self.port_handler, dxl_id, address)
        elif length == 4:
            value, comm, err = self.packet_handler.read4ByteTxRx(self.port_handler, dxl_id, address)
        else:
            print(f"[dxl] Invalid byte length: {length}")
            return None
        if not self._check(dxl_id, comm, err, "READ"):
            return None
        return _to_signed(value, length)

    # ── sync group factories ────────────────────────────────────────────────

    def make_sync_write(self, command_type):
        """Return a GroupSyncWrite bound to ``command_type`` (address, length)."""
        return GroupSyncWrite(self.port_handler, self.packet_handler, *command_type)

    def make_sync_read(self, command_type, dxl_ids):
        """Return a GroupSyncRead bound to ``command_type`` with ``dxl_ids`` registered."""
        handle = GroupSyncRead(self.port_handler, self.packet_handler, *command_type)
        for mid in dxl_ids:
            if not handle.addParam(mid):
                raise RuntimeError(f"GroupSyncRead.addParam failed for motor {mid}")
        return handle

    # ── lifecycle ───────────────────────────────────────────────────────────

    def close(self):
        """Close the serial port. Idempotent — safe to call multiple times."""
        try:
            self.port_handler.closePort()
        except Exception:
            pass

    # ── internals ───────────────────────────────────────────────────────────

    def _check(self, dxl_id, comm, err, op):
        if comm != COMM_SUCCESS:
            print(f"[dxl] {op} comm error on ID {dxl_id}: "
                  f"{self.packet_handler.getTxRxResult(comm)}")
            return False
        if err != 0:
            print(f"[dxl] {op} packet error on ID {dxl_id}: "
                  f"{self.packet_handler.getRxPacketError(err)}")
            return False
        return True


def _to_signed(value, length):
    """Interpret SDK unsigned read-back as two's-complement signed int."""
    bits = length * 8
    if value >= (1 << (bits - 1)):
        value -= 1 << bits
    return value
