NPROC := $(shell expr $$(nproc) / 2)
ifeq ($(shell expr $(NPROC) \< 1), 1)
	NPROC := 1 
endif

.PHONY: all install f clang_f cmake_f output


default: r

DATASET := ../dataset/pedestrian.jpg

install: 
	@cmake --build build --target shape_detector --parallel $(NPROC)

run:
	@./install_d/shape_detector $(DATASET)

r: clean output f install run

output:
	@mkdir -p output

clean:
	@rm -f output/*.jpg

del:
	@rm -rf build .cache output install_d

f: clang_f cmake_f
	@echo 'Format done'

clang_f:
	@clang-format -i -style=Google shape_detector.cc

cmake_f:
	@cmake-format -i CMakeLists.txt
