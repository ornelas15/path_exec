
.PHONY: all clean

all: build/CMakeCache.txt
	+ @cmake --build build -- -s

clean:
	@echo -n "Cleaning targets... "
	@[ -f build/CMakeCache.txt ] && \
	cmake --build build --target clean -- -s
	@echo "[Done]"

distclean: clean
	@echo -n "Removing build directory... "
	@[ -d build ] && rm -r build
	@echo "[Done]"

build/CMakeCache.txt:
	@[ -d build ] || mkdir -p build
	@[ -f build/CMakeCache.txt ] || cmake -B build -DCMAKE_EXPORT_COMPILE_COMMANDS=On ./

