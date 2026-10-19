@echo off
rem Start the EMIO motor GUI at http://localhost:8000 (Windows). Ctrl+C to stop.
cd /d "%~dp0"
if not exist .venv (
  py -3 -m venv .venv || python -m venv .venv
)
if not exist .venv\.installed (
  .venv\Scripts\pip install -r requirements.txt && type nul > .venv\.installed
)
if "%PORT%"=="" set PORT=8000
echo Open http://localhost:%PORT%
cd app
..\.venv\Scripts\python -m uvicorn server:app --host 127.0.0.1 --port %PORT%
pause
