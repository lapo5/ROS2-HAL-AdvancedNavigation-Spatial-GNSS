LIBNAME           = geo_to_cart_cxx

CCFLAGS           = -Wall -Wextra -Wno-narrowing -std=c++17 -shared -fPIC
INCLUDEFLAGS      = `python3 -m pybind11 --includes`
OPTIMIZATIONFLAGS = -O3 -mtune=native -march=native -DNDEBUG
DEBUGFLAGS        = -g -ggdb3 -fno-omit-frame-pointer -O0 -mtune=native -march=native
PARALLELISMFLAGS  = -fopenmp -ltbb
LDFLAGS           = -lboost_system -lboost_filesystem -lblas -llapack -DHAVE_CBLAS=1 -lfftw3 -ltbb -lGeographic

all:
	g++ ${CCFLAGS} ${INCLUDEFLAGS} ${OPTIMIZATIONFLAGS} ${PARALLELISMFLAGS} ${LIBNAME}.cpp -o ${LIBNAME}`python3-config --extension-suffix` ${LDFLAGS}

debug: dbg

dbg:
	g++ ${CCFLAGS} ${INCLUDEFLAGS} ${DEBUGFLAGS} ${LIBNAME}.cpp -o ${LIBNAME}`python3-config --extension-suffix` ${LDFLAGS} ${PARALLELISMFLAGS}

clean:
	rm -f ${LIBNAME}`python3-config --extension-suffix`
