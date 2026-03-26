import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/elena/Escritorio/LabRobotica/Proyecto-Construccion/install/piece_detection_pkg'
