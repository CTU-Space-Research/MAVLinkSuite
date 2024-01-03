"""
MAVLink protocol implementation (auto-generated by mavgen.py)

Generated from: ('csAirLink.xml',)

Note: this file has been auto-generated. DO NOT EDIT
"""
import hashlib
import json
import logging
import os
import struct
import sys
import time
from builtins import object, range
from typing import Any, Callable, Dict, Iterable, List, Mapping, Optional, Sequence, Tuple, Type, Union, cast

WIRE_PROTOCOL_VERSION = "1.0"
DIALECT = "csAirLink"

PROTOCOL_MARKER_V1 = 0xFE
PROTOCOL_MARKER_V2 = 0xFD
HEADER_LEN_V1 = 6
HEADER_LEN_V2 = 10

MAVLINK_SIGNATURE_BLOCK_LEN = 13

MAVLINK_IFLAG_SIGNED = 0x01

if sys.version_info[0] == 2:
    logging.basicConfig()

logger = logging.getLogger(__name__)

# allow MAV_IGNORE_CRC=1 to ignore CRC, allowing some
# corrupted msgs to be seen
MAVLINK_IGNORE_CRC = os.environ.get("MAV_IGNORE_CRC", 0)

# some base types from mavlink_types.h
MAVLINK_TYPE_CHAR = 0
MAVLINK_TYPE_UINT8_T = 1
MAVLINK_TYPE_INT8_T = 2
MAVLINK_TYPE_UINT16_T = 3
MAVLINK_TYPE_INT16_T = 4
MAVLINK_TYPE_UINT32_T = 5
MAVLINK_TYPE_INT32_T = 6
MAVLINK_TYPE_UINT64_T = 7
MAVLINK_TYPE_INT64_T = 8
MAVLINK_TYPE_FLOAT = 9
MAVLINK_TYPE_DOUBLE = 10


class x25crc(object):
    """CRC-16/MCRF4XX - based on checksum.h from mavlink library"""

    def __init__(self, buf: Optional[Sequence[int]] = None) -> None:
        self.crc = 0xFFFF
        if buf is not None:
            self.accumulate(buf)

    def accumulate(self, buf: Sequence[int]) -> None:
        """add in some more bytes (it also accepts python2 strings)"""
        if sys.version_info[0] == 2 and type(buf) is str:
            buf = bytearray(buf)

        accum = self.crc
        for b in buf:
            tmp = b ^ (accum & 0xFF)
            tmp = (tmp ^ (tmp << 4)) & 0xFF
            accum = (accum >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)
        self.crc = accum


class MAVLink_header(object):
    """MAVLink message header"""

    def __init__(self, msgId: int, incompat_flags: int = 0, compat_flags: int = 0, mlen: int = 0, seq: int = 0, srcSystem: int = 0, srcComponent: int = 0) -> None:
        self.mlen = mlen
        self.seq = seq
        self.srcSystem = srcSystem
        self.srcComponent = srcComponent
        self.msgId = msgId
        self.incompat_flags = incompat_flags
        self.compat_flags = compat_flags

    def pack(self, force_mavlink1: bool = False) -> bytes:
        if float(WIRE_PROTOCOL_VERSION) == 2.0 and not force_mavlink1:
            return struct.pack(
                "<BBBBBBBHB",
                254,
                self.mlen,
                self.incompat_flags,
                self.compat_flags,
                self.seq,
                self.srcSystem,
                self.srcComponent,
                self.msgId & 0xFFFF,
                self.msgId >> 16,
            )
        return struct.pack(
            "<BBBBBB",
            PROTOCOL_MARKER_V1,
            self.mlen,
            self.seq,
            self.srcSystem,
            self.srcComponent,
            self.msgId,
        )


class MAVLink_message(object):
    """base MAVLink message class"""

    id = 0
    msgname = ""
    fieldnames: List[str] = []
    ordered_fieldnames: List[str] = []
    fieldtypes: List[str] = []
    fielddisplays_by_name: Dict[str, str] = {}
    fieldenums_by_name: Dict[str, str] = {}
    fieldunits_by_name: Dict[str, str] = {}
    native_format = bytearray(b"")
    orders: List[int] = []
    lengths: List[int] = []
    array_lengths: List[int] = []
    crc_extra = 0
    unpacker = struct.Struct("")
    instance_field: Optional[str] = None
    instance_offset = -1

    def __init__(self, msgId: int, name: str) -> None:
        self._header = MAVLink_header(msgId)
        self._payload: Optional[bytes] = None
        self._msgbuf = bytearray(b"")
        self._crc: Optional[int] = None
        self._fieldnames: List[str] = []
        self._type = name
        self._signed = False
        self._link_id: Optional[int] = None
        self._instances: Optional[Dict[str, str]] = None
        self._instance_field: Optional[str] = None

    def format_attr(self, field: str) -> Union[str, float, int]:
        """override field getter"""
        raw_attr = cast(Union[bytes, float, int], getattr(self, field))
        if isinstance(raw_attr, bytes):
            if sys.version_info[0] == 2:
                return raw_attr.rstrip(b"\x00")
            return raw_attr.decode(errors="backslashreplace").rstrip("\x00")
        return raw_attr

    def get_msgbuf(self) -> bytearray:
        return self._msgbuf

    def get_header(self) -> MAVLink_header:
        return self._header

    def get_payload(self) -> Optional[bytes]:
        return self._payload

    def get_crc(self) -> Optional[int]:
        return self._crc

    def get_fieldnames(self) -> List[str]:
        return self._fieldnames

    def get_type(self) -> str:
        return self._type

    def get_msgId(self) -> int:
        return self._header.msgId

    def get_srcSystem(self) -> int:
        return self._header.srcSystem

    def get_srcComponent(self) -> int:
        return self._header.srcComponent

    def get_seq(self) -> int:
        return self._header.seq

    def get_signed(self) -> bool:
        return self._signed

    def get_link_id(self) -> Optional[int]:
        return self._link_id

    def __str__(self) -> str:
        ret = "%s {" % self._type
        for a in self._fieldnames:
            v = self.format_attr(a)
            ret += "%s : %s, " % (a, v)
        ret = ret[0:-2] + "}"
        return ret

    def __ne__(self, other: object) -> bool:
        return not self.__eq__(other)

    def __eq__(self, other: object) -> bool:
        if other is None:
            return False

        if not isinstance(other, MAVLink_message):
            return False

        if self.get_type() != other.get_type():
            return False

        if self.get_crc() != other.get_crc():
            return False

        if self.get_seq() != other.get_seq():
            return False

        if self.get_srcSystem() != other.get_srcSystem():
            return False

        if self.get_srcComponent() != other.get_srcComponent():
            return False

        for a in self._fieldnames:
            if self.format_attr(a) != other.format_attr(a):
                return False

        return True

    def to_dict(self) -> Dict[str, Union[str, float, int]]:
        d: Dict[str, Union[str, float, int]] = {}
        d["mavpackettype"] = self._type
        for a in self._fieldnames:
            d[a] = self.format_attr(a)
        return d

    def to_json(self) -> str:
        return json.dumps(self.to_dict())

    def sign_packet(self, mav: "MAVLink") -> None:
        assert mav.signing.secret_key is not None

        h = hashlib.new("sha256")
        self._msgbuf += struct.pack("<BQ", mav.signing.link_id, mav.signing.timestamp)[:7]
        h.update(mav.signing.secret_key)
        h.update(self._msgbuf)
        sig = h.digest()[:6]
        self._msgbuf += sig
        mav.signing.timestamp += 1

    def _pack(self, mav: "MAVLink", crc_extra: int, payload: bytes, force_mavlink1: bool = False) -> bytes:
        plen = len(payload)
        if float(WIRE_PROTOCOL_VERSION) == 2.0 and not force_mavlink1:
            # in MAVLink2 we can strip trailing zeros off payloads. This allows for simple
            # variable length arrays and smaller packets
            if sys.version_info[0] == 2:
                nullbyte = chr(0)
            else:
                nullbyte = 0
            while plen > 1 and payload[plen - 1] == nullbyte:
                plen -= 1
        self._payload = payload[:plen]
        incompat_flags = 0
        if mav.signing.sign_outgoing:
            incompat_flags |= MAVLINK_IFLAG_SIGNED
        self._header = MAVLink_header(
            self._header.msgId,
            incompat_flags=incompat_flags,
            compat_flags=0,
            mlen=len(self._payload),
            seq=mav.seq,
            srcSystem=mav.srcSystem,
            srcComponent=mav.srcComponent,
        )
        self._msgbuf = bytearray(self._header.pack(force_mavlink1=force_mavlink1))
        self._msgbuf += self._payload
        crc = x25crc(self._msgbuf[1:])
        if True:
            # we are using CRC extra
            crc.accumulate(struct.pack("B", crc_extra))
        self._crc = crc.crc
        self._msgbuf += struct.pack("<H", self._crc)
        if mav.signing.sign_outgoing and not force_mavlink1:
            self.sign_packet(mav)
        return bytes(self._msgbuf)

    def pack(self, mav: "MAVLink", force_mavlink1: bool = False) -> bytes:
        raise NotImplementedError("MAVLink_message cannot be serialized directly")

    def __getitem__(self, key: str) -> str:
        """support indexing, allowing for multi-instance sensors in one message"""
        if self._instances is None:
            raise IndexError()
        if key not in self._instances:
            raise IndexError()
        return self._instances[key]


class mavlink_msg_deprecated_name_property(object):
    """
    This handles the class variable name change from name to msgname for
    subclasses of MAVLink_message during a transition period.

    This is used by setting the class variable to
    `mavlink_msg_deprecated_name_property()`.
    """

    def __get__(self, instance: Optional[MAVLink_message], owner: Type[MAVLink_message]) -> str:
        if instance is not None:
            logger.error("Using .name on a MAVLink_message is not supported, use .get_type() instead.")
            raise AttributeError("Class {} has no attribute 'name'".format(owner.__name__))
        logger.warning(
            """Using .name on a MAVLink_message class is deprecated, consider using .msgname instead.
Note that if compatibility with pymavlink 2.4.30 and earlier is desired, use something like this:

msg_name =  msg.msgname if hasattr(msg, "msgname") else msg.name"""
        )
        return owner.msgname


# enums


class EnumEntry(object):
    def __init__(self, name: str, description: str) -> None:
        self.name = name
        self.description = description
        self.param: Dict[int, str] = {}
        self.has_location = False


enums: Dict[str, Dict[int, EnumEntry]] = {}

# AIRLINK_AUTH_RESPONSE_TYPE
enums["AIRLINK_AUTH_RESPONSE_TYPE"] = {}
AIRLINK_ERROR_LOGIN_OR_PASS = 0
enums["AIRLINK_AUTH_RESPONSE_TYPE"][0] = EnumEntry("AIRLINK_ERROR_LOGIN_OR_PASS", """Login or password error""")
AIRLINK_AUTH_OK = 1
enums["AIRLINK_AUTH_RESPONSE_TYPE"][1] = EnumEntry("AIRLINK_AUTH_OK", """Auth successful""")
AIRLINK_AUTH_RESPONSE_TYPE_ENUM_END = 2
enums["AIRLINK_AUTH_RESPONSE_TYPE"][2] = EnumEntry("AIRLINK_AUTH_RESPONSE_TYPE_ENUM_END", """""")

# AIRLINK_EYE_GS_HOLE_PUSH_RESP_TYPE
enums["AIRLINK_EYE_GS_HOLE_PUSH_RESP_TYPE"] = {}
AIRLINK_HPR_PARTNER_NOT_READY = 0
enums["AIRLINK_EYE_GS_HOLE_PUSH_RESP_TYPE"][0] = EnumEntry("AIRLINK_HPR_PARTNER_NOT_READY", """""")
AIRLINK_HPR_PARTNER_READY = 1
enums["AIRLINK_EYE_GS_HOLE_PUSH_RESP_TYPE"][1] = EnumEntry("AIRLINK_HPR_PARTNER_READY", """""")
AIRLINK_EYE_GS_HOLE_PUSH_RESP_TYPE_ENUM_END = 2
enums["AIRLINK_EYE_GS_HOLE_PUSH_RESP_TYPE"][2] = EnumEntry("AIRLINK_EYE_GS_HOLE_PUSH_RESP_TYPE_ENUM_END", """""")

# AIRLINK_EYE_IP_VERSION
enums["AIRLINK_EYE_IP_VERSION"] = {}
AIRLINK_IP_V4 = 0
enums["AIRLINK_EYE_IP_VERSION"][0] = EnumEntry("AIRLINK_IP_V4", """""")
AIRLINK_IP_V6 = 1
enums["AIRLINK_EYE_IP_VERSION"][1] = EnumEntry("AIRLINK_IP_V6", """""")
AIRLINK_EYE_IP_VERSION_ENUM_END = 2
enums["AIRLINK_EYE_IP_VERSION"][2] = EnumEntry("AIRLINK_EYE_IP_VERSION_ENUM_END", """""")

# AIRLINK_EYE_HOLE_PUSH_TYPE
enums["AIRLINK_EYE_HOLE_PUSH_TYPE"] = {}
AIRLINK_HP_NOT_PENETRATED = 0
enums["AIRLINK_EYE_HOLE_PUSH_TYPE"][0] = EnumEntry("AIRLINK_HP_NOT_PENETRATED", """""")
AIRLINK_HP_BROKEN = 1
enums["AIRLINK_EYE_HOLE_PUSH_TYPE"][1] = EnumEntry("AIRLINK_HP_BROKEN", """""")
AIRLINK_EYE_HOLE_PUSH_TYPE_ENUM_END = 2
enums["AIRLINK_EYE_HOLE_PUSH_TYPE"][2] = EnumEntry("AIRLINK_EYE_HOLE_PUSH_TYPE_ENUM_END", """""")

# AIRLINK_EYE_TURN_INIT_TYPE
enums["AIRLINK_EYE_TURN_INIT_TYPE"] = {}
AIRLINK_TURN_INIT_START = 0
enums["AIRLINK_EYE_TURN_INIT_TYPE"][0] = EnumEntry("AIRLINK_TURN_INIT_START", """""")
AIRLINK_TURN_INIT_OK = 1
enums["AIRLINK_EYE_TURN_INIT_TYPE"][1] = EnumEntry("AIRLINK_TURN_INIT_OK", """""")
AIRLINK_TURN_INIT_BAD = 2
enums["AIRLINK_EYE_TURN_INIT_TYPE"][2] = EnumEntry("AIRLINK_TURN_INIT_BAD", """""")
AIRLINK_EYE_TURN_INIT_TYPE_ENUM_END = 3
enums["AIRLINK_EYE_TURN_INIT_TYPE"][3] = EnumEntry("AIRLINK_EYE_TURN_INIT_TYPE_ENUM_END", """""")

# message IDs
MAVLINK_MSG_ID_BAD_DATA = -1
MAVLINK_MSG_ID_UNKNOWN = -2


mavlink_map: Dict[int, Type[MAVLink_message]] = {
}


class MAVError(Exception):
    """MAVLink error class"""

    def __init__(self, msg: str) -> None:
        Exception.__init__(self, msg)
        self.message = msg


class MAVLink_bad_data(MAVLink_message):
    """
    a piece of bad data in a mavlink stream
    """

    def __init__(self, data: bytes, reason: str) -> None:
        MAVLink_message.__init__(self, MAVLINK_MSG_ID_BAD_DATA, "BAD_DATA")
        self._fieldnames = ["data", "reason"]
        self.data = data
        self.reason = reason
        self._msgbuf = bytearray(data)
        self._instance_field = None

    def __str__(self) -> str:
        """Override the __str__ function from MAVLink_messages because non-printable characters are common in to be the reason for this message to exist."""
        if sys.version_info[0] == 2:
            hexstr = ["{:x}".format(ord(i)) for i in self.data]
        else:
            hexstr = ["{:x}".format(i) for i in self.data]
        return "%s {%s, data:%s}" % (self._type, self.reason, hexstr)


class MAVLink_unknown(MAVLink_message):
    """
    a message that we don't have in the XML used when built
    """

    def __init__(self, msgid: int, data: bytes) -> None:
        MAVLink_message.__init__(self, MAVLINK_MSG_ID_UNKNOWN, "UNKNOWN_%u" % msgid)
        self._fieldnames = ["data"]
        self.data = data
        self._msgbuf = bytearray(data)
        self._instance_field = None

    def __str__(self) -> str:
        """Override the __str__ function from MAVLink_messages because non-printable characters are common."""
        if sys.version_info[0] == 2:
            hexstr = ["{:x}".format(ord(i)) for i in self.data]
        else:
            hexstr = ["{:x}".format(i) for i in self.data]
        return "%s {data:%s}" % (self._type, hexstr)


class MAVLinkSigning(object):
    """MAVLink signing state class"""

    def __init__(self) -> None:
        self.secret_key: Optional[bytes] = None
        self.timestamp = 0
        self.link_id = 0
        self.sign_outgoing = False
        self.allow_unsigned_callback: Optional[Callable[["MAVLink", int], bool]] = None
        self.stream_timestamps: Dict[Tuple[int, int, int], int] = {}
        self.sig_count = 0
        self.badsig_count = 0
        self.goodsig_count = 0
        self.unsigned_count = 0
        self.reject_count = 0


class MAVLink(object):
    """MAVLink protocol handling class"""

    def __init__(self, file: Any, srcSystem: int = 0, srcComponent: int = 0, use_native: bool = False) -> None:
        self.seq = 0
        self.file = file
        self.srcSystem = srcSystem
        self.srcComponent = srcComponent
        self.callback: Optional[Callable[..., None]] = None
        self.callback_args: Optional[Iterable[Any]] = None
        self.callback_kwargs: Optional[Mapping[str, Any]] = None
        self.send_callback: Optional[Callable[..., None]] = None
        self.send_callback_args: Optional[Iterable[Any]] = None
        self.send_callback_kwargs: Optional[Mapping[str, Any]] = None
        self.buf = bytearray()
        self.buf_index = 0
        self.expected_length = HEADER_LEN_V1 + 2
        self.have_prefix_error = False
        self.robust_parsing = False
        self.protocol_marker = 254
        self.little_endian = True
        self.crc_extra = True
        self.sort_fields = True
        self.total_packets_sent = 0
        self.total_bytes_sent = 0
        self.total_packets_received = 0
        self.total_bytes_received = 0
        self.total_receive_errors = 0
        self.startup_time = time.time()
        self.signing = MAVLinkSigning()
        self.mav20_unpacker = struct.Struct("<cBBBBBBHB")
        self.mav10_unpacker = struct.Struct("<cBBBBB")
        self.mav20_h3_unpacker = struct.Struct("BBB")
        self.mav_csum_unpacker = struct.Struct("<H")
        self.mav_sign_unpacker = struct.Struct("<IH")

    def set_callback(self, callback: Callable[..., None], *args: Any, **kwargs: Any) -> None:
        self.callback = callback
        self.callback_args = args
        self.callback_kwargs = kwargs

    def set_send_callback(self, callback: Callable[..., None], *args: Any, **kwargs: Any) -> None:
        self.send_callback = callback
        self.send_callback_args = args
        self.send_callback_kwargs = kwargs

    def send(self, mavmsg: MAVLink_message, force_mavlink1: bool = False) -> None:
        """send a MAVLink message"""
        buf = mavmsg.pack(self, force_mavlink1=force_mavlink1)
        self.file.write(buf)
        self.seq = (self.seq + 1) % 256
        self.total_packets_sent += 1
        self.total_bytes_sent += len(buf)
        if self.send_callback is not None and self.send_callback_args is not None and self.send_callback_kwargs is not None:
            self.send_callback(mavmsg, *self.send_callback_args, **self.send_callback_kwargs)

    def buf_len(self) -> int:
        return len(self.buf) - self.buf_index

    def bytes_needed(self) -> int:
        """return number of bytes needed for next parsing stage"""
        ret = self.expected_length - self.buf_len()

        if ret <= 0:
            return 1
        return ret

    def __callbacks(self, msg: MAVLink_message) -> None:
        """this method exists only to make profiling results easier to read"""
        if self.callback is not None and self.callback_args is not None and self.callback_kwargs is not None:
            self.callback(msg, *self.callback_args, **self.callback_kwargs)

    def parse_char(self, c: Sequence[int]) -> Optional[MAVLink_message]:
        """input some data bytes, possibly returning a new message"""
        self.buf.extend(c)

        self.total_bytes_received += len(c)

        m = self.__parse_char_legacy()

        if m is not None:
            self.total_packets_received += 1
            self.__callbacks(m)
        else:
            # XXX The idea here is if we've read something and there's nothing left in
            # the buffer, reset it to 0 which frees the memory
            if self.buf_len() == 0 and self.buf_index != 0:
                self.buf = bytearray()
                self.buf_index = 0

        return m

    def __parse_char_legacy(self) -> Optional[MAVLink_message]:
        """input some data bytes, possibly returning a new message"""
        header_len = HEADER_LEN_V1
        if self.buf_len() >= 1 and self.buf[self.buf_index] == PROTOCOL_MARKER_V2:
            header_len = HEADER_LEN_V2

        m: Optional[MAVLink_message] = None
        if self.buf_len() >= 1 and self.buf[self.buf_index] != PROTOCOL_MARKER_V1 and self.buf[self.buf_index] != PROTOCOL_MARKER_V2:
            magic = self.buf[self.buf_index]
            self.buf_index += 1
            if self.robust_parsing:
                m = MAVLink_bad_data(bytearray([magic]), "Bad prefix")
                self.expected_length = header_len + 2
                self.total_receive_errors += 1
                return m
            if self.have_prefix_error:
                return None
            self.have_prefix_error = True
            self.total_receive_errors += 1
            raise MAVError("invalid MAVLink prefix '%s'" % magic)
        self.have_prefix_error = False
        if self.buf_len() >= 3:
            sbuf = self.buf[self.buf_index : 3 + self.buf_index]
            (magic, self.expected_length, incompat_flags) = cast(
                Tuple[int, int, int],
                self.mav20_h3_unpacker.unpack(sbuf),
            )
            if magic == PROTOCOL_MARKER_V2 and (incompat_flags & MAVLINK_IFLAG_SIGNED):
                self.expected_length += MAVLINK_SIGNATURE_BLOCK_LEN
            self.expected_length += header_len + 2
        if self.expected_length >= (header_len + 2) and self.buf_len() >= self.expected_length:
            mbuf = self.buf[self.buf_index : self.buf_index + self.expected_length]
            self.buf_index += self.expected_length
            self.expected_length = header_len + 2
            if self.robust_parsing:
                try:
                    if magic == PROTOCOL_MARKER_V2 and (incompat_flags & ~MAVLINK_IFLAG_SIGNED) != 0:
                        raise MAVError("invalid incompat_flags 0x%x 0x%x %u" % (incompat_flags, magic, self.expected_length))
                    m = self.decode(mbuf)
                except MAVError as reason:
                    m = MAVLink_bad_data(mbuf, reason.message)
                    self.total_receive_errors += 1
            else:
                if magic == PROTOCOL_MARKER_V2 and (incompat_flags & ~MAVLINK_IFLAG_SIGNED) != 0:
                    raise MAVError("invalid incompat_flags 0x%x 0x%x %u" % (incompat_flags, magic, self.expected_length))
                m = self.decode(mbuf)
            return m
        return None

    def parse_buffer(self, s: Sequence[int]) -> Optional[List[MAVLink_message]]:
        """input some data bytes, possibly returning a list of new messages"""
        m = self.parse_char(s)
        if m is None:
            return None
        ret = [m]
        while True:
            m = self.parse_char(b"")
            if m is None:
                return ret
            ret.append(m)

    def check_signature(self, msgbuf: bytearray, srcSystem: int, srcComponent: int) -> bool:
        """check signature on incoming message"""
        assert self.signing.secret_key is not None

        timestamp_buf = msgbuf[-12:-6]
        link_id = msgbuf[-13]
        (tlow, thigh) = cast(
            Tuple[int, int],
            self.mav_sign_unpacker.unpack(timestamp_buf),
        )
        timestamp = tlow + (thigh << 32)

        # see if the timestamp is acceptable
        stream_key = (link_id, srcSystem, srcComponent)
        if stream_key in self.signing.stream_timestamps:
            if timestamp <= self.signing.stream_timestamps[stream_key]:
                # reject old timestamp
                logger.info("old timestamp")
                return False
        else:
            # a new stream has appeared. Accept the timestamp if it is at most
            # one minute behind our current timestamp
            if timestamp + 6000 * 1000 < self.signing.timestamp:
                logger.info("bad new stream %s %s", timestamp / (100.0 * 1000 * 60 * 60 * 24 * 365), self.signing.timestamp / (100.0 * 1000 * 60 * 60 * 24 * 365))
                return False
            self.signing.stream_timestamps[stream_key] = timestamp
            logger.info("new stream")

        h = hashlib.new("sha256")
        h.update(self.signing.secret_key)
        h.update(msgbuf[:-6])
        sig1 = h.digest()[:6]
        sig2 = msgbuf[-6:]
        if sig1 != sig2:
            logger.info("sig mismatch")
            return False

        # the timestamp we next send with is the max of the received timestamp and
        # our current timestamp
        self.signing.timestamp = max(self.signing.timestamp, timestamp)
        return True

    def decode(self, msgbuf: bytearray) -> MAVLink_message:
        """decode a buffer as a MAVLink message"""
        # decode the header
        if msgbuf[0] != PROTOCOL_MARKER_V1:
            headerlen = 10
            try:
                magic, mlen, incompat_flags, compat_flags, seq, srcSystem, srcComponent, msgIdlow, msgIdhigh = cast(
                    Tuple[bytes, int, int, int, int, int, int, int, int],
                    self.mav20_unpacker.unpack(msgbuf[:headerlen]),
                )
            except struct.error as emsg:
                raise MAVError("Unable to unpack MAVLink header: %s" % emsg)
            msgId = msgIdlow | (msgIdhigh << 16)
            mapkey = msgId
        else:
            headerlen = 6
            try:
                magic, mlen, seq, srcSystem, srcComponent, msgId = cast(
                    Tuple[bytes, int, int, int, int, int],
                    self.mav10_unpacker.unpack(msgbuf[:headerlen]),
                )
                incompat_flags = 0
                compat_flags = 0
            except struct.error as emsg:
                raise MAVError("Unable to unpack MAVLink header: %s" % emsg)
            mapkey = msgId
        if (incompat_flags & MAVLINK_IFLAG_SIGNED) != 0:
            signature_len = MAVLINK_SIGNATURE_BLOCK_LEN
        else:
            signature_len = 0

        if ord(magic) != PROTOCOL_MARKER_V1 and ord(magic) != PROTOCOL_MARKER_V2:
            raise MAVError("invalid MAVLink prefix '{}'".format(hex(ord(magic))))
        if mlen != len(msgbuf) - (headerlen + 2 + signature_len):
            raise MAVError("invalid MAVLink message length. Got %u expected %u, msgId=%u headerlen=%u" % (len(msgbuf) - (headerlen + 2 + signature_len), mlen, msgId, headerlen))

        if mapkey not in mavlink_map:
            return MAVLink_unknown(msgId, msgbuf)

        # decode the payload
        msgtype = mavlink_map[mapkey]
        order_map = msgtype.orders
        len_map = msgtype.lengths
        crc_extra = msgtype.crc_extra

        # decode the checksum
        try:
            (crc,) = cast(
                Tuple[int],
                self.mav_csum_unpacker.unpack(msgbuf[-(2 + signature_len) :][:2]),
            )
        except struct.error as emsg:
            raise MAVError("Unable to unpack MAVLink CRC: %s" % emsg)
        crcbuf = msgbuf[1 : -(2 + signature_len)]
        if True:
            # using CRC extra
            crcbuf.append(crc_extra)
        crc2 = x25crc(crcbuf)
        if crc != crc2.crc and not MAVLINK_IGNORE_CRC:
            raise MAVError("invalid MAVLink CRC in msgID %u 0x%04x should be 0x%04x" % (msgId, crc, crc2.crc))

        sig_ok = False
        if signature_len == MAVLINK_SIGNATURE_BLOCK_LEN:
            self.signing.sig_count += 1
        if self.signing.secret_key is not None:
            accept_signature = False
            if signature_len == MAVLINK_SIGNATURE_BLOCK_LEN:
                sig_ok = self.check_signature(msgbuf, srcSystem, srcComponent)
                accept_signature = sig_ok
                if sig_ok:
                    self.signing.goodsig_count += 1
                else:
                    self.signing.badsig_count += 1
                if not accept_signature and self.signing.allow_unsigned_callback is not None:
                    accept_signature = self.signing.allow_unsigned_callback(self, msgId)
                    if accept_signature:
                        self.signing.unsigned_count += 1
                    else:
                        self.signing.reject_count += 1
            elif self.signing.allow_unsigned_callback is not None:
                accept_signature = self.signing.allow_unsigned_callback(self, msgId)
                if accept_signature:
                    self.signing.unsigned_count += 1
                else:
                    self.signing.reject_count += 1
            if not accept_signature:
                raise MAVError("Invalid signature")

        csize = msgtype.unpacker.size
        mbuf = msgbuf[headerlen : -(2 + signature_len)]
        if len(mbuf) < csize:
            # zero pad to give right size
            mbuf.extend([0] * (csize - len(mbuf)))
        if len(mbuf) < csize:
            raise MAVError("Bad message of type %s length %u needs %s" % (msgtype, len(mbuf), csize))
        mbuf = mbuf[:csize]
        try:
            t = cast(
                Tuple[Union[bytes, int, float], ...],
                msgtype.unpacker.unpack(mbuf),
            )
        except struct.error as emsg:
            raise MAVError("Unable to unpack MAVLink payload type=%s payloadLength=%u: %s" % (msgtype, len(mbuf), emsg))

        tlist: List[Union[bytes, float, int, Sequence[float], Sequence[int]]] = list(t)
        # handle sorted fields
        if True:
            if sum(len_map) == len(len_map):
                # message has no arrays in it
                for i in range(0, len(tlist)):
                    tlist[i] = t[order_map[i]]
            else:
                # message has some arrays
                tlist = []
                for i in range(0, len(order_map)):
                    order = order_map[i]
                    L = len_map[order]
                    tip = sum(len_map[:order])
                    field = t[tip]
                    if L == 1 or isinstance(field, bytes):
                        tlist.append(field)
                    else:
                        tlist.append(cast(Union[Sequence[int], Sequence[float]], list(t[tip : (tip + L)])))

        # terminate any strings
        for i, elem in enumerate(tlist):
            if isinstance(elem, bytes):
                tlist[i] = elem.rstrip(b"\x00")

        # construct the message object
        try:
            # Note that initializers don't follow the Liskov Substitution Principle
            # therefore it can't be typechecked
            m = msgtype(*tlist)  # type: ignore
        except Exception as emsg:
            raise MAVError("Unable to instantiate MAVLink message of type %s : %s" % (msgtype, emsg))
        m._signed = sig_ok
        if m._signed:
            m._link_id = msgbuf[-13]
        m._msgbuf = msgbuf
        m._payload = msgbuf[6 : -(2 + signature_len)]
        m._crc = crc
        m._header = MAVLink_header(msgId, incompat_flags, compat_flags, mlen, seq, srcSystem, srcComponent)
        return m
