Dumping buffers with GDB:
1. run application for long enough to fill the buffers
2. halt application (ctrl+c)
3. dump memory:
   $> dump binary memory impulse_response.bin response (response+256)
4. run plotting script (plot_buffer.py)
