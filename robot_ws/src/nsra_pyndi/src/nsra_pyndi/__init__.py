"""
Mini library used to read video data from an NDI source
"""

from nsra_pyndi.lib import lib, ffi
from nsra_pyndi.finder import create_ndi_finder
from nsra_pyndi.receiver import create_receiver

# for typings
from nsra_pyndi.finder import NDIFinder, NDISource


