Dumping buffers with GDB:
1. run application for long enough to fill the buffers
2. halt application (ctrl+c)
3. dump memory:
   $> dump binary memory adaptive_filter_coeffs.bin fir_coeffs_f32 (fir_coeffs_f32+256)
   Note: there is an alias defined to do this in ./.gdbinit: dump-coeffs
4. run plotting script (plot_buffer.py)

NOTE: make sure right channel output is looped back to right channel input!
