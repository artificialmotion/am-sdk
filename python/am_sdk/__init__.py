# SPDX-License-Identifier: Apache-2.0
"""Artificial Motion joint SDK for Python.

Same call names as the C header (c/include/am_api.h). Backends: the built-in simulator
("sim:M,H-F"), EtherCAT ("ecat:<nic>", needs pysoem) and CAN-FD ("can:<if>:<channel>",
needs python-can). Your controller code is the same on all of them.
"""
from .am_types import *  # noqa: F401,F403
from .am_types import (AmStatus, AmError, AmBus, AmMode, AmDriveState, AmSku, AmRefState, AmDataBasis, AmEventKind,
                       AmIdentity, AmTransparency, AmCommand, AmImpedance, AmTelemetry, AmLimits, AmLoopGains,
                       AmFaultInfo, AmFaultRecord, AmDeviceInfo, AmEvent, AmSessionConfig, am_status_str,
                       am_fault_name, am_version_pack)
from .api import *  # noqa: F401,F403
from .api import AmSession, AmJoint
from .codec import (AmCodec, am_codec_init, am_codec_pack_rx1600, am_codec_pack_rx1601, am_codec_unpack_tx1a00,
                    am_codec_unpack_tx1a01, am_codec_pack_cfd_slot, am_codec_unpack_cfd_fb, am_codec_decode_state,
                    am_codec_next_controlword)
from .backends import (Backend, JointBackend, SimBackend, SimJoint, SimOptions, SimTruth, TruthPlant, SdoAbort,
                       Transport, TransportBackend, SimWireTransport)
from .units import UNIT_IDS, load_unit

__version__ = "0.2.0.dev0"
