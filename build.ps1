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
    single_file_tests_run = $false; single_file_test_count = 0; single_file_failures = $null
    single_file_scenario_matches = $false
}
$report | ConvertTo-Json | Set-Content -LiteralPath $reportFile -Encoding utf8
'' | Set-Content -LiteralPath $logFile -Encoding utf8
# Never leave an old success XML beside a failed current build.
$xmlPath = Join-Path $resultDir 'gtest.xml'
$singleXmlPath = Join-Path $resultDir 'gtest_single.xml'
foreach ($taskXml in @($xmlPath, $singleXmlPath)) {
    if (Test-Path -LiteralPath $taskXml) { Remove-Item -LiteralPath $taskXml }
}
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
    Copy-Item -LiteralPath 'build/cmake/gtest_single.xml' -Destination $singleXmlPath -Force
    [xml]$singleTestXml = Get-Content -LiteralPath $singleXmlPath -Raw
    $report.single_file_tests_run = $true
    $report.single_file_test_count = [int]$singleTestXml.testsuites.tests
    $report.single_file_failures = [int]$singleTestXml.testsuites.failures + [int]$singleTestXml.testsuites.errors
    $report.phase = 'scenario'
    $demoOutput = & '.\build\cmake\rvc_demo.exe' --scenario 'scenarios/demo.csv' 2>&1
    $demoExit = $LASTEXITCODE
    $demoOutput | Set-Content -LiteralPath (Join-Path $resultDir 'demo_run.csv') -Encoding utf8
    if ($demoExit -ne 0) { throw "Scenario failed with exit code $demoExit." }
    $singleOutput = & '.\build\cmake\rvc_single_demo.exe' --scenario 'scenarios/demo.csv' 2>&1
    $singleExit = $LASTEXITCODE
    $singleOutput | Set-Content -LiteralPath (Join-Path $resultDir 'single_demo_run.csv') -Encoding utf8
    if ($singleExit -ne 0) { throw "Single-file scenario failed with exit code $singleExit." }
    if (($demoOutput -join "`n") -cne ($singleOutput -join "`n")) {
        throw 'Single-file scenario differs from the modular program.'
    }
    $report.single_file_scenario_matches = $true
    $report.status = 'passed'
    $report.phase = 'complete'
    $report.compiler = $gcc
    Write-Output ("Implemented C17 RVC: {0} GoogleTest cases, {1} failures; scenario passed." -f $report.test_count, $report.failures)
    Write-Output ("Combined C17 RVC: {0} GoogleTest cases, {1} failures; same scenario output." -f $report.single_file_test_count, $report.single_file_failures)
} catch {
    $report.status = 'failed'
    $report.error = $_.Exception.Message
    throw
} finally {
    $report | ConvertTo-Json | Set-Content -LiteralPath $reportFile -Encoding utf8
    $env:PATH = $oldTaskPath
    Pop-Location
}
