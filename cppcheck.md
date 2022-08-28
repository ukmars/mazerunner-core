CPPCHECK the code with

cppcheck --enable=all --force -q --language=c++  *.cpp *.h *.ino 2>&1 | less


run in the src folder