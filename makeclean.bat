@echo off
del /S /Q *.exe
del /S /Q *.obj
del /S /Q *.o
del /S /Q *.ncb
del /S /Q *.sdf
del /S /Q *.filters
del /S /Q /AH *.suo
del /S /Q *.user
del /S /Q *.ilk
del /S /Q *.pdb
del /S /Q build\debug\temp\*.*
del /S /Q build\release\temp\*.*
rmdir /S /Q Release