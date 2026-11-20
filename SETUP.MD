# Setup Guide: From Zero to Your First Run

This guide is for people who have never programmed before. It takes you from a fresh Windows PC to
running the simulation, changing the car and getting the newest version. Every step says what to
click or type. You do not need to understand Rust to use the simulation: you edit text files and
run one command.

If something goes wrong, look at [section 11, Common errors](#11-common-errors) first.

---

## 1. What You Will Install

| Program | What it is for |
|---|---|
| **Rust** (with `rustup` and `cargo`) | Turns the code into a program and runs it |
| **Visual Studio C++ Build Tools** | Windows needs these for Rust to make a program. The Rust installer offers them |
| **Git** (or GitHub Desktop) | Downloads the project and later the newest changes |
| **VS Code** (or RustRover) | An editor to open the files and type commands in one window |

All of them are free. Set aside about 30 to 45 minutes and about 8 GB of disk space, most of it for
the Build Tools.

---

## 2. Install Rust

1. Go to <https://rustup.rs> and download **rustup-init.exe** (64-bit).
2. Run it. A black window opens.
3. If it says the Visual Studio C++ prerequisites are missing, choose
   **1) Quick install via the Visual Studio Community installer**. The Visual Studio Installer
   opens. Leave the ticked items as they are (they include **Desktop development with C++** and a
   Windows SDK) and click **Install**. Wait until it finishes, then go back to the black window.
4. When it asks how to install Rust, press **Enter** for the default installation.
5. When it says *Rust is installed now*, close the window.
6. **Close and reopen** every terminal and VS Code window you have open, so they find Rust.

Check that it works. Press the Windows key, type `PowerShell`, open it and type:

```
cargo --version
```

You should see something like `cargo 1.91.0`. The project needs Rust **1.85 or newer**. If
your version is older, run `rustup update`.

---

## 3. Install Git

**Option A: Git (recommended).** Download it from <https://git-scm.com/download/win> and run the
installer. Clicking **Next** on every page with the defaults is fine. Check it in a new PowerShell
window:

```
git --version
```

**Option B: GitHub Desktop**, if you prefer buttons to typing. Download it from
<https://desktop.github.com> and sign in with your GitHub account. It contains its own Git.

**Option C: no Git at all.** You can download the project as a ZIP (section 5, option C), but then
you have to download it again for every update. Use this only to try it once.

---

## 4. Install an Editor

### VS Code (recommended)

1. Download it from <https://code.visualstudio.com> and install it with the defaults.
2. Open VS Code, click the **Extensions** icon on the left (four squares), search for
   **rust-analyzer** and click **Install**. It shows errors, types and explanations when you hover
   over the code.
3. Optional but handy: **Even Better TOML** (colours for the `.toml` data files) and
   **Rainbow CSV** (colours the columns of the track and log files).

### RustRover (option)

RustRover from JetBrains is a full Rust IDE, free for non-commercial use. Download it from
<https://www.jetbrains.com/rust/>. It works the same way: open the project folder, and use its
**Terminal** tab for the commands in this guide.

---

## 5. Download the Project (Clone)

You only do this once. "Cloning" makes a copy of the project on your PC that Git can update later.

Pick a folder for it, for example `C:\Users\<you>\Projects`.

**Option A: Git in PowerShell**

```
cd C:\Users\<you>\Projects
git clone https://github.com/Schuteros/F1_digital_twin.git
cd F1_digital_twin
```

**Option B: GitHub Desktop.** **File > Clone repository > URL**, paste
`https://github.com/Schuteros/F1_digital_twin`, pick the folder and click **Clone**.

**Option C: ZIP.** On <https://github.com/Schuteros/F1_digital_twin> click the green **Code**
button, then **Download ZIP**, and unzip it. The folder will be called `F1_digital_twin-master`.

Now open it in VS Code: **File > Open Folder...** and choose the `F1_digital_twin` folder (the one
that contains `Cargo.toml`). If VS Code asks whether you trust the authors, click **Yes**.

Open a terminal inside VS Code with **Terminal > New Terminal** (or `` Ctrl+` ``). It starts in the
project folder, which is where every command below must be typed.

---

## 6. Build and Run

Type this in the terminal:

```
cargo run --release -- data/runs/baseline.toml
```

What the parts mean:

* `cargo run` builds the program and starts it.
* `--release` makes a fast build. Without it the program is much slower to run.
* `--` separates the options for `cargo` from the options for the simulation. Everything after it
  goes to the simulation.
* `data/runs/baseline.toml` is the **run file**: it says which car, tyres, track and conditions to
  simulate.

The **first** time takes a few minutes: cargo downloads the libraries the project uses (you need
internet) and builds everything. After that it only rebuilds what changed, usually in seconds.

At the end you see the results in the terminal, for example the final time, speed and distance, and
on a track with XY points the lap length and every lap time.

Try the other example runs:

```
cargo run --release -- data/runs/skidpad_5_laps.toml
cargo run --release -- data/runs/oval_3_laps.toml
```

**Checking that everything works:** `cargo test` runs all the automatic checks of the code. Every
line should end in `ok`. You don't need it for normal use, but it is a good check after an update.

---

## 7. Where Things Are

```
F1_digital_twin/
  data/          all the files you edit: cars, tyres, tracks, environments, runs
    private/     your team's real data, never uploaded (see below)
  docs/          the physics, one document per topic (01 to 21), with the equations
  src/           the Rust code. You don't need to touch it
  target/        what cargo builds. Safe to delete, it is rebuilt
  README.MD      what the simulation does now, how a run works, the to-do list
  SETUP.MD       this guide
```

The data files, what each key means and its unit, are explained in
**[data/README.md](data/README.md)**. Read that next. In short:

| Folder | File type | What it holds |
|---|---|---|
| `data/cars/` | `.toml` | Mass, geometry, aero, brakes, engine and gearbox, suspension |
| `data/tyres/` | `.toml` | Tyre friction and wheel radius. Swappable, so one car can try several tyres |
| `data/tracks/` | `.csv` | Either XY points of the line the car drives (`x_m,y_m`) or braking zones on a straight (`start_m,end_m`) |
| `data/environments/` | `.toml` | Air density and gravity |
| `data/runs/` | `.toml` | Picks one car, tyre, track and environment, plus the time step, when to stop and how the car starts |

TOML files are plain text: `key = value`, with sections in `[square brackets]` and comments after
`#`. Every key has its unit in its name, for example `total_kg`, `wheelbase_m` or
`spring_rate_N_per_m`. Capital letters matter: `N` is newton, `n_per_m` is an error.

### The `data/private/` folder

`data/private/` is for **real team data** (our car, measured tyres, real tracks). Git ignores this
folder, so nothing in it is ever uploaded to GitHub, and it is **not** in what you download either.
Get the team files from the team (not from GitHub) and put them in `data/private/`, with the same
layout as `data/`:

```
data/private/
  cars/  tyres/  tracks/  environments/  runs/
```

Run them the same way:

```
cargo run --release -- data/private/runs/<run file>.toml
```

Rules: never copy real team values into the files outside `data/private/`, into the docs or into
anything you share publicly. Once something is uploaded to GitHub it stays in the history even after
it is deleted.

---

## 8. Make Your Own Run

Don't edit the example files themselves. Copy them, so the examples stay as they are and updates
(section 10) don't clash with your changes.

**Example: the skidpad with a heavier car.**

1. In VS Code's file list, right-click `data/cars/baseline.toml`, **Copy**, then right-click the
   `data/cars` folder, **Paste**. Rename the copy to `heavy.toml`.
2. Open `heavy.toml` and change the mass, for example `total_kg`. `front_kg + rear_kg` must still add
   up to `total_kg`, so change those too.
3. Copy `data/runs/skidpad_5_laps.toml` to `data/runs/my_skidpad.toml`.
4. In `my_skidpad.toml`, point the car line at your file:

   ```toml
   car = "../cars/heavy.toml"
   ```

   Paths inside a file are **relative to that file's folder**. `..` means "one folder up", so from
   `data/runs/` the path `../cars/heavy.toml` goes to `data/cars/heavy.toml`.
5. Run it and compare the lap times with the original:

   ```
   cargo run --release -- data/runs/my_skidpad.toml
   ```

The things you will change most often in a run file:

* `car`, `tyres`, `track`, `environment`: which files to use. `tyres` is optional, without it the
  car's `default_tyres` is used.
* `end_condition`: `{ time_s = 30.0 }` stops after 30 s, `{ laps = 3, max_time_s = 600.0 }` stops
  after 3 laps (only on a track with XY points).
* `time_step_s`: the size of one simulation step. `0.001` (1 ms) is a good value. Larger is faster
  but can make the suspension unstable, which the program tells you.
* `[driver]`: how carefully the driver drives, for example `grip_margin = 0.05` to keep 5 % grip in
  reserve.
* `[initial_state]`: the starting speed, gear and position.

Every key is listed in [data/README.md](data/README.md). The files themselves have comments where a
key needs one.

**Your own track:** make a `.csv` file with the header `x_m,y_m` and one point per line in driving
order, about 1 m apart, without repeating the first point at the end. The details are in
[data/README.md](data/README.md), section *Track file*. A spreadsheet can save this format with
**Save as > CSV (Comma delimited)**, but check that it uses a dot `.` for decimals (see section 9).

---

## 9. Output and Logs

The results are printed in the terminal. To get the car state at every step in a file (for plots in
Excel, Python, MATLAB, ...), add `--log`:

```
cargo run --release -- data/runs/oval_3_laps.toml --log logs/oval.csv --log-every 10
```

* `--log logs/oval.csv` writes a CSV file. The path is relative to the folder the terminal is in
  (the project folder), not to the run file. The folder (`logs` here) must already exist: create it
  first with `mkdir logs` or in the file list. An existing file with the same name is overwritten.
* `--log-every 10` writes every 10th step, so the file is 10 times smaller. Without it every step is
  written.

The first row names each column with its unit: time, distance, speed, acceleration, gear, engine
speed, suspension motion, lap, x and y position, corner speed limit, target speed, lateral
acceleration, tyre loads and more. [README.MD](README.MD) section 2 lists them.

**Opening the log in Excel:** the file uses a dot for decimals and a comma between columns. If your
Windows uses a comma for decimals (Latvian and most European settings do), double-clicking the file
gives wrong numbers. Instead, open an empty workbook and use **Data > From Text/CSV**, choose the
file, and if the numbers look wrong set the file origin / locale to **English (United States)**.

---

## 10. Get the Newest Version

The project changes often. To update your copy:

**Git in the terminal** (in the project folder):

```
git pull
```

**GitHub Desktop:** click **Fetch origin**, then **Pull origin**.

**ZIP:** download a new ZIP (section 5, option C). Copy your own files (your runs, cars and the
whole `data/private/` folder) from the old folder into the new one.

Then run as usual. `cargo run` rebuilds the changed code by itself. The newest changes are described
at the top of [README.MD](README.MD) (*Latest Milestone*). A new version sometimes adds a required key
to the data files. The example files are updated with it, but your own copies are not: the error
message names the missing key, and [data/README.md](data/README.md) says what it means.

Your own new files (like `my_skidpad.toml`) and everything in `data/private/` are never touched by
an update. Only files that came with the project are updated, which is why section 8 says to copy
them instead of editing them.

---

## 11. Common Errors

**`cargo : The term 'cargo' is not recognized...`**
The terminal was open before Rust was installed. Close every terminal and VS Code, open them again.
If it still fails, restart the PC.

**`linker 'link.exe' not found`** or **`error: linking with link.exe failed`**
The Visual Studio C++ Build Tools are missing. Open the **Visual Studio Installer** from the Start
menu, click **Modify**, tick **Desktop development with C++** and install. Then run again.

**`feature 'edition2024' is required`** or **`requires rustc 1.85`**
Your Rust is too old. Run `rustup update`.

**`could not find Cargo.toml in ...`**
The terminal is not in the project folder. Type `cd ` followed by the path of the folder that
contains `Cargo.toml`, or open that folder in VS Code and open a new terminal.

**`Error: ...` followed by `Usage: cargo run -- <run file> ...`**
The program did not get a run file, or got a wrong option. Check that there is `--` before the run
file and that the path is right. Use `/` or `\` in paths, both work.

**`Error: data\runs\my_run.toml: cannot read file: ...`**
The file does not exist at that path. Check the spelling and the folder. Remember that paths
inside a run file are relative to the run file's folder (section 8).

**`Error: <file>: unknown field 'spring_rate_n_per_m'`** or **`missing field ...`**
A key in that file is misspelled (often the capital letter of a unit), not allowed in that section,
or missing. The message names the file and the key. Compare with [data/README.md](data/README.md) or
the example files.

**`Error: <file>: invalid config: ...`**
All keys are there but a value is not allowed, for example `front_kg + rear_kg` is not equal to
`total_kg`, a `front_bias` above 1, or a time step too large for the suspension. The message says
which value and why.

**`Error: ... TOML parse error ...`**
The file is not valid TOML: often a missing `"` around a path, a `,` instead of `.` in a number,
or a section name without `[ ]`. The message gives the line and column.

**Error naming a `.csv` file and a line number**
A bad row in a track file. Usually a comma used for decimals (`1,5` instead of `1.5`), an empty
value, or text in a number column. Lines starting with `#` are comments and are fine.

**`--log` fails with *The system cannot find the path specified***
The folder for the log file does not exist. Create it first (section 9).

**`git pull` says *Your local changes ... would be overwritten by merge***
You edited a file that came with the project. Save your version under a new name, then undo the
change with `git checkout -- <file>` (or in GitHub Desktop: right-click the file, **Discard
changes**) and pull again.

**The simulation is very slow**
You probably left out `--release`. Without it the program is built for debugging and runs many times
slower.

**Something else**
Copy the whole error message (the first error is usually the important one) and send it to the
person who maintains the project, together with the command you typed and the run file you used.
