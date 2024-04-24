GCOV_FLAGS= --coverage
GCOV_LIBS= -lgcov

DEBUG_BUILD_TYPE=-DCMAKE_BUILD_TYPE=Debug
GCOV_REPORT_FLAGS=-DGRAPH_COVERAGE_FLAGS:STRING="$(GCOV_FLAGS)"
GCOV_REPORT_LIBS=-DGRAPH_COVERAGE_LIBS:STRING="$(GCOV_LIBS)"
RELEASE_BUILD_TYPE=-DCMAKE_BUILD_TYPE=Release
GCOV_NO_REPORT_FLAGS=-DGRAPH_COVERAGE_FLAGS:STRING=""
GCOV_NO_REPORT_LIBS=-DGRAPH_COVERAGE_LIBS:STRING=""

REPORT_BUILD= $(DEBUG_BUILD_TYPE) $(GCOV_REPORT_FLAGS) $(GCOV_REPORT_LIBS)
STANDART_BUILD= $(DEBUG_BUILD_TYPE) $(GCOV_NO_REPORT_FLAGS) $(GCOV_NO_REPORT_LIBS)


PATH_BUILD=build
PATH_REPORT=report
PATH_DOXY=doxy

.PHONY: all install uninstall clean rebuild test gcov_report valgrind leaks run

all: install 

install: init_submodules
	cmake -B $(PATH_BUILD)
	cmake --build $(PATH_BUILD) --target all 

run:
	./build/simple_navigator

uninstall:
	rm -rf $(PATH_BUILD)

clean:
	rm -rf $(PATH_REPORT)

rebuild: clean all

test: build_test
	$(PATH_BUILD)/simple_navigator_test

valgrind: test
	valgrind $(PATH_BUILD)/simple_navigator_test

gcov_report: build_test_cov
	rm -rf $(PATH_REPORT)
	./$(PATH_BUILD)/simple_navigator_test
	mkdir $(PATH_REPORT)	
	gcovr --html-details -o $(PATH_REPORT)/index.html
	open $(PATH_REPORT)/index.html

init_submodules:
	git submodule init
	git submodule update

build_test: init_submodules
	cmake -B $(PATH_BUILD) $(STANDART_BUILD) 
	cmake --build $(PATH_BUILD) --target simple_navigator_test

build_test_cov: init_submodules
	cmake -B $(PATH_BUILD) $(REPORT_BUILD) 
	cmake --build $(PATH_BUILD) --target simple_navigator_test

