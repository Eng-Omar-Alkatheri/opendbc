from dataclasses import dataclass, field
from enum import IntFlag
import re

from opendbc.car import Bus, CarSpecs, DbcDict, PlatformConfig, Platforms
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.structs import CarParams
from opendbc.car.docs_definitions import CarHarness, CarDocs, CarParts
from opendbc.car.fw_query_definitions import FwQueryConfig, LiveFwVersions, OfflineFwVersions, Request, StdQueries

Ecu = CarParams.Ecu


# Steer torque limits

class CarControllerParams:
  STEER_MAX = 800                # theoretical max_steer 2047
  STEER_DELTA_UP = 10             # torque increase per refresh
  STEER_DELTA_DOWN = 25           # torque decrease per refresh
  STEER_DRIVER_ALLOWANCE = 15     # allowed driver torque before start limiting
  STEER_DRIVER_MULTIPLIER = 1     # weight driver torque
  STEER_DRIVER_FACTOR = 1         # from dbc
  STEER_STEP = 1  # 100 Hz

  def __init__(self, CP):
    pass


@dataclass
class MazdaCarDocs(CarDocs):
  package: str = "All"
  car_parts: CarParts = field(default_factory=CarParts.common([CarHarness.mazda]))


@dataclass(frozen=True, kw_only=True)
class MazdaCarSpecs(CarSpecs):
  tireStiffnessFactor: float = 0.7  # not optimized yet


class MazdaFlags(IntFlag):
  # Static flags
  # Gen 1 hardware: same CAN messages and same camera
  GEN1 = 1


@dataclass
class MazdaPlatformConfig(PlatformConfig):
  dbc_dict: DbcDict = field(default_factory=lambda: {Bus.pt: 'mazda_2017'})
  flags: int = MazdaFlags.GEN1


class CAR(Platforms):
  MAZDA_CX5 = MazdaPlatformConfig(
    [MazdaCarDocs("Mazda CX-5 2017-21")],
    MazdaCarSpecs(mass=3655 * CV.LB_TO_KG, wheelbase=2.7, steerRatio=15.5)
  )
  MAZDA_CX9 = MazdaPlatformConfig(
    [MazdaCarDocs("Mazda CX-9 2016-20")],
    MazdaCarSpecs(mass=4217 * CV.LB_TO_KG, wheelbase=3.1, steerRatio=17.6)
  )
  MAZDA_3 = MazdaPlatformConfig(
    [MazdaCarDocs("Mazda 3 2017-18")],
    MazdaCarSpecs(mass=2875 * CV.LB_TO_KG, wheelbase=2.7, steerRatio=14.0)
  )
  MAZDA_6 = MazdaPlatformConfig(
    [MazdaCarDocs("Mazda 6 2017-20")],
    MazdaCarSpecs(mass=3443 * CV.LB_TO_KG, wheelbase=2.83, steerRatio=15.5)
  )
  MAZDA_CX9_2021 = MazdaPlatformConfig(
    [MazdaCarDocs("Mazda CX-9 2021-23", video="https://youtu.be/dA3duO4a0O4")],
    MAZDA_CX9.specs
  )
  MAZDA_CX5_2022 = MazdaPlatformConfig(
    [MazdaCarDocs("Mazda CX-5 2022-25")],
    MAZDA_CX5.specs,
  )


class LKAS_LIMITS:
  STEER_THRESHOLD = 15
  DISABLE_SPEED = 45    # kph
  ENABLE_SPEED = 52     # kph


class Buttons:
  NONE = 0
  SET_PLUS = 1
  SET_MINUS = 2
  RESUME = 3
  CANCEL = 4


# GCC (Saudi-market) Mazda fingerprinting support:
# Mazda FW strings are part numbers of the form  <PREFIX>-<FAMILY>-<REV>
# e.g.  PXM6-188K2-C (engine), GSH7-67XK2-U (camera), TA0B-437K2-C (ABS).
# The PREFIX identifies the vehicle program (PXM* = CX-9 turbo, PX2* = CX-5,
# TC3M = CX-9 2021 EPS, ...) and is the real model discriminator. The FAMILY
# identifies the functional group (188K2 = engine, 67XK2 = ADAS, 437K2 = ABS,
# 21PS1 = transmission, 3210X = EPS) and is shared across all Mazda models.
# Regional variants (GCC/Japan/Europe) share prefixes+families with US cars but
# carry new revision letters, so exact string match fails. Matching on
# (prefix, family) — ignoring the revision — is GCC-tolerant while prefix-level
# contradiction checks keep models separated.
_MAZDA_FW_PATTERN = re.compile(rb'^([A-Z0-9]{2,6})-([A-Z0-9]{4,7})-')


def _mazda_fw_key(fw_version: bytes) -> tuple[bytes, bytes] | None:
  """Extract (prefix, family) from a Mazda part number: PXM6-188K2-C -> (PXM6, 188K2)."""
  match = _MAZDA_FW_PATTERN.match(fw_version.strip(b'\x00 '))
  if match is None:
    return None
  prefix, family = match.group(1), match.group(2)
  return (prefix, family)


def match_fw_to_car_fuzzy(live_fw_versions: LiveFwVersions, vin: str,
                          offline_fw_versions: OfflineFwVersions) -> set[str]:
  """Mazda-specific fuzzy match tolerant of GCC/regional FW variants.

  A candidate matches if it has no contradictions and at least 2 distinct
  ECU addresses strong-match. Returns all matching candidates; the caller
  requires exactly one.
  """
  # prefix -> platforms carrying it (any ECU)
  prefix_platforms: dict[bytes, set[str]] = {}
  for candidate, fws in offline_fw_versions.items():
    cstr = str(candidate)
    for (_ecu, _addr, _sub), versions in fws.items():
      for v in versions:
        key = _mazda_fw_key(v)
        if key is not None:
          prefix_platforms.setdefault(key[0], set()).add(cstr)

  candidates: set[str] = set()
  for candidate, fws in offline_fw_versions.items():
    cstr = str(candidate)
    strong_addrs: set[tuple[int, int | None]] = set()
    contradict = False

    expected_pf = set()
    for (_ecu, addr, sub), versions in fws.items():
      for v in versions:
        key = _mazda_fw_key(v)
        if key is not None:
          expected_pf.add((addr, sub, key))

    for live_addr, live_versions in live_fw_versions.items():
      keys = {k for k in (_mazda_fw_key(v) for v in live_versions) if k is not None}
      if not keys:
        continue

      # contradiction: prefix known, but never on this platform
      for prefix, family in keys:
        platforms = prefix_platforms.get(prefix, set())
        if platforms and cstr not in platforms:
          contradict = True
          break

        # strong: (prefix, family) exists for this candidate at this address
        if (live_addr[0], live_addr[1], (prefix, family)) in expected_pf:
          strong_addrs.add(live_addr)
      if contradict:
        break

    if not contradict and len(strong_addrs) >= 2:
      candidates.add(cstr)

  return candidates


FW_QUERY_CONFIG = FwQueryConfig(
  fw_version_regex=br"[A-Z0-9-]{11,16}\x00{8,13}",
  requests=[
    # TODO: check data to ensure ABS does not skip ISO-TP frames on bus 0
    Request(
      [StdQueries.MANUFACTURER_SOFTWARE_VERSION_REQUEST],
      [StdQueries.MANUFACTURER_SOFTWARE_VERSION_RESPONSE],
      bus=0,
    ),
  ],
  # GCC-market Mazdas carry regional part numbers; use family-based fuzzy match
  match_fw_to_car_fuzzy=match_fw_to_car_fuzzy,
)

DBC = CAR.create_dbc_map()
