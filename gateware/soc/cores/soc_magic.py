"""Read-only identity registers for the daemon's startup check.

The daemon reads ``soc_magic_value`` before it configures anything and asserts
it matches. A mismatch means the link is wedged, the wrong or a half-loaded
bitstream is present, or the CSR base is wrong, so the daemon fails fast
instead of driving a SoC it is not really talking to.

``soc_version_value`` moves whenever the CSR layout changes, which lets a
daemon built against an older map refuse to run rather than write registers
that have moved.
"""

from litex.gen import LiteXModule
from litex.soc.interconnect.csr import CSRStatus

# ASCII "PSKS". The value is arbitrary but fixed, and both the gateware and the
# daemon hold the same constant.
SOC_MAGIC = 0x50534B53
SOC_VERSION = 3


class _Constant(LiteXModule):
    """A fixed value in one read-only CSR, ``value``."""

    def __init__(self, value, description):
        self.value = CSRStatus(32, description=description)
        self.comb += self.value.status.eq(value)


def add_identity(soc, magic=SOC_MAGIC, version=SOC_VERSION):
    """Adds ``soc_magic_value`` and ``soc_version_value`` to ``soc``."""
    soc.soc_magic = _Constant(magic, "Fixed product identity confirming a coherent SoC.")
    soc.soc_version = _Constant(version, "CSR layout version.")
