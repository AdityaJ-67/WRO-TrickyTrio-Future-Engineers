"""Pi 5 'Brain': vision, mission state machine, planning, serial master.

Nothing in this package may block for longer than a control period, and
nothing in it is trusted by the Pico. If this whole process dies, the Pico
stops the vehicle on its own 300 ms link timeout -- that is the contract.
"""
