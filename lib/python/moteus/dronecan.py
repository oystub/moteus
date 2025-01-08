import asyncio
import dronecan
import os


class CanMessage:
    arbitration_id = 0
    is_extended_id = False
    dlc = 0
    data = b''
    is_fd = False
    bitrate_switch = False


# Implements a 'Transport' on top of a dronecan.Tunnel
class DronecanTransport:
    def __init__(self, *args, **kwargs):
        self.node = dronecan.make_node(*args, **kwargs)
        self.node.node_info.name = "moteus"
        self.node.node_info.hardware_version.unique_id = os.urandom(16)
        self.node.mode = dronecan.uavcan.protocol.NodeStatus().MODE_OPERATIONAL

        self.reply_queue = asyncio.Queue()
        self.cycle_lock = asyncio.Lock()

        self.node.add_handler(dronecan.uavcan.tunnel.Broadcast, self.handle_Tunnel)

        asyncio.create_task(self.spin_node())

    def handle_Tunnel(self, msg):
        try:
            payload = msg.message.buffer.to_bytes()
            if len(payload) < 2:
                print("Invalid message received: too short")
                return None

            asyncio.run_coroutine_threadsafe(self.reply_queue.put(payload), asyncio.get_event_loop())
        except Exception as e:
            print(f"Error handling Tunnel message: {e}")

        return None

    async def cycle(self, commands):
        async with self.cycle_lock:
            result = []
            for x in commands:
                reply = await self._do_command(x)
                if reply is not None:
                    result.append(reply)
            return result

    async def _do_command(self, command):
        await self.write(command)

        if not command.reply_required:
            return None

        try:
            reply = await asyncio.wait_for(self.wait_for_reply(), timeout=0.1)
            msg = CanMessage()
            msg.data = reply[:2]
            return command.parse(msg)
        except asyncio.TimeoutError:
            print("Timeout waiting for reply")
            return None

    async def write(self, command, cycle=False):
        msg = dronecan.uavcan.tunnel.Broadcast()
        msg.protocol.protocol = 255  # UAVCAN_TUNNEL_PROTOCOL_UNDEFINED

        source = b'\x80' if command.reply_required else b'\x00'

        # (command.can_prefix << 16)) Ignore for now

        msg.buffer = source + bytes([command.destination]) + command.data
        self.node.broadcast(msg)

    async def read(self):
        # Read a single CAN message and do not parse it.
        try:
            reply = await asyncio.wait_for(self.wait_for_reply(), timeout=0.1)
        except asyncio.TimeoutError:
            print("Timeout waiting for read")
            return None
        # Rebuild the arbritration_id
        # It has destination in the lower 8 bits, and source in the next 8 bits
        message = CanMessage()
        message.arbitration_id = reply[0] + (reply[1] << 8)
        message.data = reply[2:]
        return message

    async def wait_for_reply(self):
        while True:
            # Wait until a message appears in the queue
            reply = await self.reply_queue.get()
            return reply

    async def spin_node(self):
        while True:
            try:
                self.node.spin(0.001)
                await asyncio.sleep(0.001)  # Allow other tasks to run
            except Exception as e:
                print(f"Error in node spin: {e}")
