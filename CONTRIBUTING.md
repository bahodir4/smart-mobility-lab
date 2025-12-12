# Contributing

This project is primarily for coursework, but contributions are welcome.

## How to contribute
1. Fork the repo
2. Create a feature branch:
   ```bash
   git checkout -b feature/my-change
   ```
3. Make changes and keep them minimal and readable
4. Test locally (at minimum):
   ```bash
   python3 -m compileall .
   python3 tb3_auto.py --help
   ```
5. Open a Pull Request with:
   - What you changed
   - Why you changed it
   - How you tested it

## Style
- Keep modules small and focused
- Prefer clear logging and safe defaults
- Avoid hardcoding paths; use config.yaml when possible
