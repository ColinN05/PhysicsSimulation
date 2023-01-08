g++ src/**.cpp -I src -I dependencies/glew/include -Ldependencies/glew -lglew32s -lglew32s -lopengl32 -lgdi32 -o Application
if NOT %ERRORLEVEL% == 0 PAUSE
if %ERRORLEVEL% == 0 Application