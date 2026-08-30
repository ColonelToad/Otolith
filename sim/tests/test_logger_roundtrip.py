"""Python round-trip + cross-language byte-layout checks for the log contract."""

import struct
import tempfile
from pathlib import Path

import numpy as np


def test_python_round_trip():
    import sys
    sys.path.insert(0, "sim")
    from otolith_sim.logger import LogWriter, read_log, LogRow, HEADER_SIZE, ROW_BYTES

    with tempfile.TemporaryDirectory() as td:
        p = Path(td) / "py.bin"
        dt = 0.002
        rows_in = []
        for i in range(5):
            rows_in.append(LogRow(
                t=i*dt,
                gyro=np.array([0.01*i, 0.02*i, 0.03*i]),
                accel=np.array([0.1, 9.81, 0.0]),
                qj=np.linspace(0, 1, 12) + i*0.01,
                contacts=np.array([1,0,1,0], dtype=np.uint8),
                gt_contacts=np.array([1,0,1,0], dtype=np.uint8),
                gt_pos=np.array([0.2*i, 0.0, 0.27]),
                gt_quat=np.array([1.0, 0.0, 0.0, 0.0]),
                gt_vel=np.array([0.2, 0.0, 0.0]),
                gt_rpy_rate=np.array([0.0, 0.0, 0.0]),
                gt_accel=np.array([0.0, 0.0, 0.0]),
            ))
        with LogWriter(p, dt) as w:
            for r in rows_in:
                w.write(r)

        # file size check
        assert p.stat().st_size == HEADER_SIZE + 5*ROW_BYTES

        dt2, rows_out = read_log(p)
        assert dt2 == dt
        assert len(rows_out) == 5
        for a, b in zip(rows_in, rows_out):
            assert a.t == b.t
            np.testing.assert_allclose(a.gyro, b.gyro)
            np.testing.assert_allclose(a.qj, b.qj)
            assert np.array_equal(a.contacts, b.contacts)
            np.testing.assert_allclose(a.gt_pos, b.gt_pos)


def test_header_magic_and_offsets():
    """Byte-level layout matches the C++ header."""
    import sys
    sys.path.insert(0, "sim")
    from otolith_sim.logger import LogWriter, MAGIC, VERSION, ROW_BYTES, HEADER_SIZE, LogRow
    import numpy as np, tempfile
    from pathlib import Path

    with tempfile.TemporaryDirectory() as td:
        p = Path(td) / "hdr.bin"
        with LogWriter(p, dt=0.002) as w:
            w.write(LogRow(
                t=0.0, gyro=np.zeros(3), accel=np.zeros(3), qj=np.zeros(12),
                contacts=np.zeros(4, dtype=np.uint8), gt_contacts=np.zeros(4, dtype=np.uint8),
                gt_pos=np.zeros(3), gt_quat=np.array([1,0,0,0], float),
                gt_vel=np.zeros(3), gt_rpy_rate=np.zeros(3), gt_accel=np.zeros(3)))

        raw = p.read_bytes()
        assert raw[:4] == MAGIC
        magic, version, row_bytes, reserved, dt, count = struct.unpack("<4s I I I d Q", raw[:32])
        assert magic == MAGIC
        assert version == VERSION
        assert row_bytes == ROW_BYTES
        assert dt == 0.002
        assert count == 1


def test_cross_lang_python_writes_cpp_reads():
    """Python writes log, C++ log_check reads it (proves contract interop)."""
    import subprocess, sys
    sys.path.insert(0, "sim")
    from otolith_sim.logger import LogWriter, LogRow
    import numpy as np, tempfile
    from pathlib import Path
    # skip if C++ binary not built
    checker = Path("fusion/build/log_check")
    if not checker.exists():
        import pytest
        pytest.skip("fusion/build/log_check not built")
    with tempfile.TemporaryDirectory() as td:
        p = Path(td) / "cross.bin"
        dt = 0.002
        with LogWriter(p, dt) as w:
            for i in range(7):
                w.write(LogRow(t=i*dt, gyro=np.array([0.11*i,0,0]), accel=np.array([0,0,9.81]),
                               qj=np.zeros(12), contacts=np.array([1,0,1,0],dtype=np.uint8),
                               gt_contacts=np.array([1,0,1,0],dtype=np.uint8),
                               gt_pos=np.array([0.1*i,0,0.27]), gt_quat=np.array([1,0,0,0],float),
                               gt_vel=np.zeros(3), gt_rpy_rate=np.zeros(3), gt_accel=np.zeros(3)))
        out = subprocess.check_output([str(checker), str(p)], text=True)
        n, dts = out.splitlines()[0].split()
        assert int(n) == 7
        assert float(dts) == dt


def test_puppet_logger_smoke():
    """record_puppet_log produces a readable file."""
    import sys
    sys.path.insert(0, "sim")
    from otolith_sim.logger import read_log, record_puppet_log
    import tempfile
    from pathlib import Path
    with tempfile.TemporaryDirectory() as td:
        p = Path(td) / "puppet.bin"
        record_puppet_log(p, duration_s=0.02, dt=1/500)
        dt, rows = read_log(p)
        assert len(rows) == 10  # 0.02 * 500
        assert dt == 1/500
        # contacts are booleans 0/1, qps plausible
        assert rows[0].qj.shape == (12,)
