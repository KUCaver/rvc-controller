param([string]$ToolchainBin = 'C:\msys64\ucrt64\bin')
$ErrorActionPreference = 'Stop'
$taskRoot = $PSScriptRoot
$oldTaskPath = $env:PATH
$resultDir = Join-Path $taskRoot 'results'
New-Item -ItemType Directory -Force -Path $resultDir | Out-Null
$reportFile = Join-Path $resultDir 'implementation_check.json'
$logFile = Join-Path $resultDir 'build.log'
$report = [ordered]@{
    status = 'running'; phase = 'configure'; implementation_date = '2026-10-03'
    product_language = 'C17'; test_language = 'C++17'
    fsm_implemented = $true; behavior_tests_run = $false
    google_test_integrated = $true; test_count = 0; failures = $null
}
$report | ConvertTo-Json | Set-Content -LiteralPath $reportFile -Encoding utf8
'' | Set-Content -LiteralPath $logFile -Encoding utf8
# Never leave an old success XML beside a failed current build.
$xmlPath = Join-Path $resultDir 'gtest.xml'
if (Test-Path -LiteralPath $xmlPath) { Remove-Item -LiteralPath $xmlPath }
function Invoke-Checked([string]$Program, [string[]]$Arguments, [string]$Phase) {
    $script:report.phase = $Phase
    $output = & $Program @Arguments 2>&1
    $code = $LASTEXITCODE
    $output | Add-Content -LiteralPath $script:logFile -Encoding utf8
    $output | Write-Output
    if ($code -ne 0) { throw "$Phase failed with exit code $code. See results/build.log." }
}
Push-Location -LiteralPath $taskRoot
try {
    $env:PATH = $ToolchainBin + [IO.Path]::PathSeparator + $env:PATH
    $cmake = (Get-Command cmake -ErrorAction Stop).Source
    $ctest = (Get-Command ctest -ErrorAction Stop).Source
    $gcc = (Get-Command gcc -ErrorAction Stop).Source.Replace('\','/')
    $gxx = (Get-Command g++ -ErrorAction Stop).Source.Replace('\','/')
    $rvcBin = $ToolchainBin.Replace('\','/')
    Invoke-Checked $cmake @('-S','.', '-B','build/cmake','-G','Ninja',
        "-DCMAKE_C_COMPILER=$gcc", "-DCMAKE_CXX_COMPILER=$gxx",
        "-DCMAKE_RC_COMPILER=$rvcBin/windres.exe",
        "-DCMAKE_MAKE_PROGRAM=$rvcBin/ninja.exe",
        '-DCMAKE_BUILD_TYPE=Debug','-DBUILD_TESTING=ON') 'configure'
    Invoke-Checked $cmake @('--build','build/cmake','--parallel','4') 'compile'
    Invoke-Checked $ctest @('--test-dir','build/cmake','--output-on-failure') 'test'
    Copy-Item -LiteralPath 'build/cmake/gtest.xml' -Destination $xmlPath -Force
    [xml]$testXml = Get-Content -LiteralPath $xmlPath -Raw
    $report.behavior_tests_run = $true
    $report.test_count = [int]$testXml.testsuites.tests
    $report.failures = [int]$testXml.testsuites.failures + [int]$testXml.testsuites.errors
    $report.phase = 'scenario'
    $demoOutput = & '.\build\cmake\rvc_demo.exe' --scenario 'scenarios/demo.csv' 2>&1
    $demoExit = $LASTEXITCODE
    $demoOutput | Set-Content -LiteralPath (Join-Path $resultDir 'demo_run.csv') -Encoding utf8
    if ($demoExit -ne 0) { throw "Scenario failed with exit code $demoExit." }
    $report.status = 'passed'
    $report.phase = 'complete'
    $report.compiler = $gcc
    Write-Output ("Implemented C17 RVC: {0} GoogleTest cases, {1} failures; scenario passed." -f $report.test_count, $report.failures)
} catch {
    $report.status = 'failed'
    $report.error = $_.Exception.Message
    throw
} finally {
    $report | ConvertTo-Json | Set-Content -LiteralPath $reportFile -Encoding utf8
    $env:PATH = $oldTaskPath
    Pop-Location
}
