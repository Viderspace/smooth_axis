# smooth_axis Test Suite

## First time setup:

```bash
# From project root
make prepare-tests
```

**This will:**

* **Install Python dependencies (matplotlib, pandas, numpy)**
* **Create test data directories**
* **Compile all test executables**

---

## Run tests and generate plots:

```bash
# From project root
make run-tests
```

**This will:**

1. Run ramp response tests → generates CSV files in - tests/data/ramp_files/
2. Run step response tests → generates CSV files in - tests/data/step_files/
3. Run API sanity tests → prints 29 test results to console
4. Generate visualization plots → saves PNG files to ```tests/data/renders/```

---

## See the renders

```
## Directory Structure

tests/
├── data/
│   ├── ramp_files/  
│   ├── step_files/  
│   └── renders/      (generated) <------- HERE ----------------------
├── py_scripts/
│   ├── plot_ramp.py   
│   ├── plot_step.py         
│   ├── requirements.txt     
│   └── README.md           
.
.
```

---
<details>
<summary>Manual Compilation</summary>

## Test Files

- **ramp_response_test.c** - Tests settle time accuracy across environmental conditions
- **step_response_test.c** - Tests step response and 95% threshold detection
- **test_api_sanity_enhanced.c** - 29 API edge case and safety tests

---

## Manual Compilation (From project root)

If you need to compile tests manually (without Make):

#### Ramp test

```bash
gcc -Wall -I./src -o build/ramp_test tests/ramp_response_test.c src/smooth_axis.c -lm
```

#### Step test

```bash
gcc -Wall -I./src -o build/step_test tests/step_response_test.c src/smooth_axis.c -lm
```

#### API test (must use -DNDEBUG for release mode)

```bash
gcc -Wall -DNDEBUG -I./src -o build/test_api tests/test_api_sanity_enhanced.c src/smooth_axis.c -lm
```

#### Run tests (must run from project root!)

```bash
./build/ramp_test
./build/step_test
./build/test_api
```

#### Generate plots

```bash
python tests/py_scripts/plot_ramp.py
python tests/py_scripts/plot_step.py
```

</details>