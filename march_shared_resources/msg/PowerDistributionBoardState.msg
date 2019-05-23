bool high_voltage_enabled

# The current consumed by the power distribution board.
float32 power_distribution_board_current

# The total current of all high voltage nets combined.
float32 high_voltage_net_current

# The request of the power distribution board to shutdown the master.
bool master_shutdown_requested

LowVoltageNet[] low_voltage_nets
HighVoltageNet[] high_voltage_nets

