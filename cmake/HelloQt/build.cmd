@echo off
set arg1=%1
set source_dir=C:\Qt\6.5.1\msvc2019_64\bin
set source_dir_plugin=C:\Qt\6.5.1\msvc2019_64\plugins\renderers
set destination_dir=C:\Users\liqui\Desktop\Colosseum\cmake\output\bin\Debug
set qgl_dir=C:\libQGLViewer\libQGLViewer-2.9.1\QGLViewer

cmake -G "Visual Studio 17 2022" -DCMAKE_BUILD_TYPE=Debug .. 
call cmake --build . -j16 
if not errorlevel 1 (
    for %%f in ("%source_dir%\*.dll") do (
        copy "%%f" "%destination_dir%\"
    )
    for  %%f in ("%source_dir_plugin%\*.dll") do (
		copy "%%f" "%destination_dir%\"
	)
    for  %%f in ("%qgl_dir%\QGLViewer2.dll" "%qgl_dir%\QGLViewerd2.dll") do (
		copy "%%f" "%destination_dir%\"
	)
)



