CXX      := g++
CXXFLAGS := -std=c++17 -Wall -Wextra -g \
            -DVWEGOLF_NATIVE_TEST \
            -Imock \
            -I. \
            -I../src \
            -include mock/mock_ovms.hpp

SRC := mock/mock_ovms.cpp \
       ../src/vehicle_vwegolf.cpp \
       ../src/vwegolf_tx.cpp \
       test_can_decode.cpp \
       test_crtd_replay.cpp \
       test_clima.cpp \
       test_uds_poll.cpp

all: test

test: $(SRC)
	$(CXX) $(CXXFLAGS) $^ -o $@
	./$@

# DBC-decode regression suite. Asserts the cantools decode of vwegolf.dbc
# matches the per-capture .md ground-truth claims. Requires the project venv
# (.venv) with cantools+pytest installed — see analysis/README.md.
# Activate .venv first or pass PYTEST=/path/to/.venv/bin/pytest.
PYTEST ?= pytest
pytest:
	cd analysis && $(PYTEST) -q

# Regenerates the committed synthetic CRTD fixtures (candumps/kcan-synthetic.crtd,
# candumps/can3-synthetic.crtd) from docs/vwegolf.dbc. Requires the project venv
# (.venv) with cantools installed — see analysis/README.md. Not a prerequisite of
# `test`: the C++ gate stays dependency-free (no cantools, no Python).
PYTHON ?= python
fixtures:
	cd analysis && $(PYTHON) make_synthetic_crtd.py

clean:
	rm -f test

.PHONY: all test pytest fixtures clean
