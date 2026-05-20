@echo off
setlocal

echo Upgrading pip...
py -3.12 -m pip install --upgrade pip

echo Installing project requirements...
py -3.12 -m pip install -r requirements.txt

echo.
echo Installation complete.
echo Run the app with:
echo py -3.12 main.py

endlocal
