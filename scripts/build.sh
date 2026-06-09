ls /opt/unitree_robotics/lib/ && \
make -f Makefile.sharpa_bridge \
UNITREE_SDK2_DIR=/opt/unitree_robotics \
SHARPA_INCLUDE_DIR=../sdk/include \
SHARPA_LIB_DIR=../sdk/lib 2>&1 | tail -40
