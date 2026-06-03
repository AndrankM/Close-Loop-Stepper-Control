<#
.SYNOPSIS
  Redeploy the led_app to the Raspberry Pi 5 and restart the service.

.DESCRIPTION
  Copies app.py and templates/index.html to the Pi, then restarts the
  led_app systemd service. Prompts for the SSH/sudo password interactively.

.EXAMPLE
  ./redeploy.ps1
  ./redeploy.ps1 -PiHost 192.168.0.103 -PiUser andpi5
#>
param(
    [string]$PiHost = "192.168.0.103",
    [string]$PiUser = "andpi5",
    [string]$RemoteDir = "/home/andpi5/led_app"
)

$ErrorActionPreference = "Stop"
$target = "$PiUser@$PiHost"
$here = Split-Path -Parent $MyInvocation.MyCommand.Path

Write-Host "==> Uploading files to $target ..." -ForegroundColor Cyan
scp "$here\app.py" "$here\templates\index.html" "${target}:/tmp/"

Write-Host "==> Moving files into place and restarting service ..." -ForegroundColor Cyan
# -t allocates a TTY so sudo can prompt for its password.
# Single-line command avoids CRLF issues from PowerShell here-strings.
$remote = "mv /tmp/app.py $RemoteDir/app.py && mv /tmp/index.html $RemoteDir/templates/index.html && sudo systemctl restart led_app && sleep 3 && systemctl is-active led_app && curl -s http://127.0.0.1:5000/motor/status; echo"
ssh -t -o StrictHostKeyChecking=no $target $remote

Write-Host "==> Done. UI: http://${PiHost}:5000" -ForegroundColor Green
