$devlibs_url = "http://guildofwriters.org/tools/devlibs.zip"

if (!(Test-Path -PathType Container build)) {
    Write-Host "Creating build folder... " -noNewLine
    New-Item -ItemType directory build | Out-Null
    Write-Host "OK" -foregroundColor Green
}
Set-Location build
$path = (Get-Location).Path

if (!(Test-Path -PathType Container devlibs)) {
    Write-Host "Downloading development libraries... " -noNewLine
    ## This only works in PS3+ so we'll download the file directly
    # Invoke-WebRequest $devlibs_url -OutFile devlibs.zip
    $client = New-Object System.Net.WebClient
    $client.DownloadFile($devlibs_url, $path + "\devlibs.zip")
    Write-Host "OK" -foregroundColor Green

    Write-Host "Extracting development libraries... " -noNewLine
    New-Item -ItemType directory devlibs | Out-Null

    $shell_app = New-Object -com shell.application
    $zip  = $shell_app.namespace($path + "\devlibs.zip")
    $dest = $shell_app.namespace($path + "\devlibs")
    $dest.CopyHere($zip.items(), 0x14)
    Write-Host "OK" -foregroundColor Green
}

Write-Host "Running CMake to configure build system... "
cmake -DCMAKE_INSTALL_PREFIX=devlibs -G "Visual Studio 12" ..

if ($Host.Name -eq "ConsoleHost") {
    Write-Host ""
    Write-Host "Press any key to continue..."
    $Host.UI.RawUI.ReadKey("NoEcho,IncludeKeyUp") > $null
}
