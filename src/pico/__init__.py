"""Hardware drivers for the Pico 2 W 'Brainstem'.

Every module here is deliberately dumb: it knows how to talk to one chip and
nothing about the mission. All policy -- when to read, what a reading means,
what to do about a bad one -- lives in pico/main.py (micro-policy) or on the
Pi 5 (macro-policy). Keeping the drivers free of mission logic is what makes
them testable on the bench with a REPL and three wires.

MicroPython note: these are copied FLAT onto the Pico's filesystem by
tools/deploy_pico.sh (see README). On the Pico they import as `tca9548a`,
not `drivers.tca9548a`, so nothing here may use package-relative imports.
"""
