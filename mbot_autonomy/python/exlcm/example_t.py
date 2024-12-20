"""LCM type definitions
This file automatically generated by lcm.
DO NOT MODIFY BY HAND!!!!
"""

try:
    import cStringIO.StringIO as BytesIO
except ImportError:
    from io import BytesIO
import struct

import mbot_lcm_msgs.particle_t

class example_t(object):
    __slots__ = ["utime", "num_particles", "particles"]

    __typenames__ = ["int64_t", "int32_t", "mbot_lcm_msgs.particle_t"]

    __dimensions__ = [None, None, None]

    def __init__(self):
        self.utime = 0
        self.num_particles = 0
        self.particles = mbot_lcm_msgs.particle_t()

    def encode(self):
        buf = BytesIO()
        buf.write(example_t._get_packed_fingerprint())
        self._encode_one(buf)
        return buf.getvalue()

    def _encode_one(self, buf):
        buf.write(struct.pack(">qi", self.utime, self.num_particles))
        assert self.particles._get_packed_fingerprint() == mbot_lcm_msgs.particle_t._get_packed_fingerprint()
        self.particles._encode_one(buf)

    def decode(data):
        if hasattr(data, 'read'):
            buf = data
        else:
            buf = BytesIO(data)
        if buf.read(8) != example_t._get_packed_fingerprint():
            raise ValueError("Decode error")
        return example_t._decode_one(buf)
    decode = staticmethod(decode)

    def _decode_one(buf):
        self = example_t()
        self.utime, self.num_particles = struct.unpack(">qi", buf.read(12))
        self.particles = mbot_lcm_msgs.particle_t._decode_one(buf)
        return self
    _decode_one = staticmethod(_decode_one)

    def _get_hash_recursive(parents):
        if example_t in parents: return 0
        newparents = parents + [example_t]
        tmphash = (0x1db4623114406c+ mbot_lcm_msgs.particle_t._get_hash_recursive(newparents)) & 0xffffffffffffffff
        tmphash  = (((tmphash<<1)&0xffffffffffffffff) + (tmphash>>63)) & 0xffffffffffffffff
        return tmphash
    _get_hash_recursive = staticmethod(_get_hash_recursive)
    _packed_fingerprint = None

    def _get_packed_fingerprint():
        if example_t._packed_fingerprint is None:
            example_t._packed_fingerprint = struct.pack(">Q", example_t._get_hash_recursive([]))
        return example_t._packed_fingerprint
    _get_packed_fingerprint = staticmethod(_get_packed_fingerprint)

    def get_hash(self):
        """Get the LCM hash of the struct"""
        return struct.unpack(">Q", example_t._get_packed_fingerprint())[0]

