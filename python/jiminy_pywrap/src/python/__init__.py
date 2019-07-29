###############################################################################
## @brief             Entry point for jiminy_pywrap python module.
##
## @copyright         Wandercraft
###############################################################################

from wdc.redirect_stdio import RedirectStdErr as _RedirectStdErr
with _RedirectStdErr():
    from .libjiminy_pywrap import *

# We remove the NONBLOCK flag which is set on stdin file by the logger when importing a wdc C++ library.
from wdc.redirect_stdio import reset_stdin_flag as _reset_stdin_flag
_reset_stdin_flag()
