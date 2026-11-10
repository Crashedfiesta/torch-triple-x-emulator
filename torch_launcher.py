#!/usr/bin/env python3
"""Graphical launcher for the Torch Triple X emulator (no emulator changes needed)."""

import json
import os
from pathlib import Path
import subprocess
import sys
import tkinter as tk
from tkinter import filedialog, messagebox, ttk

APP_TITLE = "Torch Triple X Launcher"
SCRIPT_DIR = Path(__file__).resolve().parent
CONFIG_DIR = Path.home() / ".config" / "torch-triple-x" if os.name != "nt" else Path(os.environ.get("APPDATA", Path.home())) / "TorchTripleX"
CONFIG_FILE = CONFIG_DIR / "launcher.json"
DEFAULTS = {
    "emulator": "triplex.exe" if os.name == "nt" else "triplex",
    "rom": "triplex.rom", "disk": "HD00.img", "disk4": "",
    "unix_floppy": "newblank5.img", "keydisk": "torch_KEY.imd",
    "tap": "tap0", "network": True, "host": True, "sdl": True,
    "scale": "1", "verbose": False,
}

class TorchLauncher:
    def __init__(self, root):
        self.root = root
        self.root.title(APP_TITLE)
        self.root.minsize(650, 480)
        self.root.resizable(True, False)
        self.proc = None
        self.vars = {}
        state = dict(DEFAULTS)
        try:
            with CONFIG_FILE.open("r", encoding="utf-8") as f:
                loaded = json.load(f)
                if isinstance(loaded, dict):
                    state.update({k: v for k, v in loaded.items() if k in state})
        except (OSError, ValueError, TypeError):
            pass
        for key, value in state.items():
            self.vars[key] = tk.BooleanVar(value=value) if isinstance(DEFAULTS[key], bool) else tk.StringVar(value=str(value))

        style = ttk.Style()
        if "clam" in style.theme_names():
            style.theme_use("clam")
        style.configure("Heading.TLabel", font=("TkDefaultFont", 15, "bold"))
        style.configure("Section.TLabelframe.Label", font=("TkDefaultFont", 10, "bold"))

        frame = ttk.Frame(root, padding=16)
        frame.pack(fill="both", expand=True)
        ttk.Label(frame, text="Torch Triple X Emulator", style="Heading.TLabel").pack(anchor="w")
        ttk.Label(frame, text="Choose your media and start the machine.").pack(anchor="w", pady=(2, 4))
        ttk.Label(frame, text="Press F12 for disk manager", foreground="#346b94").pack(anchor="w", pady=(0, 14))

        media = ttk.LabelFrame(frame, text="Emulator and disk images", padding=10, style="Section.TLabelframe")
        media.pack(fill="x")
        media.columnconfigure(1, weight=1)
        fields = [
            ("Emulator", "emulator", False),
            ("Caretaker ROM", "rom", False),
            ("Hard disk (ID 0)", "disk", False),
            ("Second hard disk (ID 4)", "disk4", True),
            ("Unix floppy", "unix_floppy", True),
            ("Key disk (IMD)", "keydisk", True),
        ]
        for i, (label, key, optional) in enumerate(fields):
            ttk.Label(media, text=label + (" (optional)" if optional else "")).grid(row=i, column=0, sticky="w", pady=4, padx=(0, 10))
            ttk.Entry(media, textvariable=self.vars[key]).grid(row=i, column=1, sticky="ew", pady=4)
            ttk.Button(media, text="Browse…", command=lambda k=key: self.browse(k)).grid(row=i, column=2, padx=(8, 0), pady=4)

        options = ttk.LabelFrame(frame, text="Startup options", padding=10, style="Section.TLabelframe")
        options.pack(fill="x", pady=(12, 0))
        options.columnconfigure(3, weight=1)
        ttk.Checkbutton(options, text="Host 68010", variable=self.vars["host"]).grid(row=0, column=0, sticky="w", padx=(0, 18))
        ttk.Checkbutton(options, text="SDL display", variable=self.vars["sdl"]).grid(row=0, column=1, sticky="w", padx=(0, 18))
        ttk.Checkbutton(options, text="Verbose logging", variable=self.vars["verbose"]).grid(row=0, column=2, sticky="w")
        ttk.Label(options, text="Display scale").grid(row=1, column=0, sticky="w", pady=(12, 0))
        ttk.Combobox(options, textvariable=self.vars["scale"], values=("1", "2", "3", "4"), state="readonly", width=5).grid(row=1, column=1, sticky="w", pady=(12, 0))
        ttk.Checkbutton(options, text="Enable TAP networking", variable=self.vars["network"]).grid(row=2, column=0, columnspan=2, sticky="w", pady=(12, 0))
        ttk.Label(options, text="TAP interface").grid(row=2, column=2, sticky="e", padx=(8, 5), pady=(12, 0))
        ttk.Entry(options, textvariable=self.vars["tap"], width=16).grid(row=2, column=3, sticky="w", pady=(12, 0))

        buttons = ttk.Frame(frame)
        buttons.pack(fill="x", pady=(16, 0))
        self.status = tk.StringVar(value="Ready")
        ttk.Label(buttons, textvariable=self.status).pack(side="left")
        self.start_button = ttk.Button(buttons, text="Start Emulator", command=self.launch)
        self.start_button.pack(side="right")
        ttk.Button(buttons, text="Save Settings", command=lambda: self.save(show_confirmation=True)).pack(side="right", padx=(0, 8))
        root.protocol("WM_DELETE_WINDOW", self.close)
        self.poll_process()

    def browse(self, key):
        value = self.vars[key].get().strip()
        candidate = Path(value).expanduser()
        if not candidate.is_absolute():
            candidate = SCRIPT_DIR / candidate
        initial_dir = candidate.parent if candidate.parent.is_dir() else SCRIPT_DIR
        filetypes = [("All files", "*")]
        if key == "keydisk":
            filetypes = [("ImageDisk files", "*.imd"), ("All files", "*")]
        elif key == "rom":
            filetypes = [("ROM files", "*.rom"), ("All files", "*")]
        result = filedialog.askopenfilename(parent=self.root, title="Select " + key.replace("_", " "), initialdir=str(initial_dir), filetypes=filetypes)
        if result:
            chosen = Path(result)
            try:
                self.vars[key].set(str(chosen.relative_to(SCRIPT_DIR)))
            except ValueError:
                self.vars[key].set(str(chosen))

    def resolve(self, value):
        path = Path(value).expanduser()
        return path if path.is_absolute() else SCRIPT_DIR / path

    def save(self, show_confirmation=False):
        try:
            CONFIG_DIR.mkdir(parents=True, exist_ok=True)
            data = {name: var.get() for name, var in self.vars.items()}
            with CONFIG_FILE.open("w", encoding="utf-8") as f:
                json.dump(data, f, indent=2)
            if show_confirmation:
                self.status.set("Settings saved")
            return True
        except OSError as exc:
            messagebox.showerror("Save failed", f"Could not save settings:\n{exc}", parent=self.root)
            return False

    def launch(self):
        if self.proc is not None and self.proc.poll() is None:
            messagebox.showinfo("Already running", "The emulator is already running.", parent=self.root)
            return
        required = ("emulator", "rom", "disk")
        media = ("disk4", "unix_floppy", "keydisk")
        for key in (*required, *media):
            value = self.vars[key].get().strip()
            if not value and key in required:
                messagebox.showerror("Missing file", f"Please select a {key} file.", parent=self.root)
                return
            if value and not self.resolve(value).is_file():
                messagebox.showerror("File not found", f"Cannot find {key}:\n{self.resolve(value)}", parent=self.root)
                return
        if self.vars["network"].get() and not self.vars["tap"].get().strip():
            messagebox.showerror("Missing TAP name", "Enter a TAP interface name or disable networking.", parent=self.root)
            return
        cmd = [str(self.resolve(self.vars["emulator"].get().strip())), "--rom", str(self.resolve(self.vars["rom"].get().strip())), "--disk", str(self.resolve(self.vars["disk"].get().strip()))]
        for key, arg in (("disk4", "--disk4"), ("unix_floppy", "--unix-floppy"), ("keydisk", "--keydisk")):
            value = self.vars[key].get().strip()
            if value:
                cmd.extend((arg, str(self.resolve(value))))
        if self.vars["host"].get():
            cmd.append("--host")
        if self.vars["sdl"].get():
            cmd.extend(("--sdl", "--scale", self.vars["scale"].get()))
        if self.vars["network"].get():
            cmd.extend(("--tap", self.vars["tap"].get().strip()))
        if self.vars["verbose"].get():
            cmd.append("--verbose")
        if not self.save():
            return
        try:
            self.proc = subprocess.Popen(cmd, cwd=SCRIPT_DIR)
        except OSError as exc:
            messagebox.showerror("Could not start emulator", str(exc), parent=self.root)
            return
        self.status.set(f"Emulator running (PID {self.proc.pid})")
        self.start_button.configure(state="disabled")

    def poll_process(self):
        if self.proc is not None and self.proc.poll() is not None:
            code = self.proc.returncode
            self.proc = None
            self.status.set(f"Emulator exited (code {code})")
            self.start_button.configure(state="normal")
        self.root.after(500, self.poll_process)

    def close(self):
        # Closing the launcher never kills a running emulator.
        self.save()
        self.root.destroy()

if __name__ == "__main__":
    app = tk.Tk()
    TorchLauncher(app)
    app.mainloop()
