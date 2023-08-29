:: Clean Visual Studio Solution
rmdir /s /q .vs
echo Delete .vs Folder

:: Clean x64 folder
rmdir /s /q .\x64
rmdir /s /q .\00_ClearCmd\x64
rmdir /s /q .\01_ReadEncoder\x64
rmdir /s /q .\02_PTPGenerator\x64
rmdir /s /q .\03_PTPControl\x64
rmdir /s /q .\04_Tracking\x64
:: Add here after new project creates
:: rmdir /s /q .\[New_Project_Name]\x64
echo Delete All x64 Folders

pause
