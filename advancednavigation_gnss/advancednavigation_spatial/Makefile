LIBNAME           = advancednavigation_spatial

CCFLAGS           = -Wall -Wextra -Wno-narrowing -std=c++17 -shared -fPIC
INCLUDEFLAGS      = `python3 -m pybind11 --includes`
OPTIMIZATIONFLAGS = -O3 -mtune=native -march=native -DNDEBUG
PYBIND11FLAGS     = -shared -fPIC `python3 -m pybind11 --includes` -o ${LIBNAME}`python3-config --extension-suffix`

SOURCES_LIB = spatialsdk/an_packet_protocol.c \
              spatialsdk/spatial_packets.c \
              spatialsdk/rs232/rs232.c

all:
	g++ ${CCFLAGS} ${INCLUDEFLAGS} ${OPTIMIZATIONFLAGS} ${LIBNAME}.cpp ${SOURCES_LIB} -o ${LIBNAME}`python3-config --extension-suffix`

clean:
	rm -f ${LIBNAME}`python3-config --extension-suffix`