param(
    [string]$Source = "EKF_Explanation.tex",
    [string]$OutputDir = "."
)

$ErrorActionPreference = "Stop"

if (-not (Test-Path -LiteralPath $Source)) {
    throw "Source file '$Source' was not found."
}

if (-not (Test-Path -LiteralPath $OutputDir)) {
    New-Item -ItemType Directory -Path $OutputDir | Out-Null
}

$pdflatex = Get-Command pdflatex -ErrorAction SilentlyContinue
$tectonic = Get-Command tectonic -ErrorAction SilentlyContinue

if ($pdflatex) {
    Write-Host "Building PDF with pdflatex..."
    & $pdflatex.Path "-interaction=nonstopmode" "-halt-on-error" "-output-directory=$OutputDir" $Source
    & $pdflatex.Path "-interaction=nonstopmode" "-halt-on-error" "-output-directory=$OutputDir" $Source
    Write-Host "PDF build complete."
    exit 0
}

if ($tectonic) {
    Write-Host "Building PDF with tectonic..."
    & $tectonic.Path "--outdir" $OutputDir $Source
    Write-Host "PDF build complete."
    exit 0
}

throw "No LaTeX engine was found. Install 'pdflatex' or 'tectonic', then rerun this script."
