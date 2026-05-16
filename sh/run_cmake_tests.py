#!/usr/bin/env python3

import os
import re
import subprocess
import sys

# ANSI color codes for terminal
GREEN = "\033[92m"
RED = "\033[91m"
YELLOW = "\033[93m"
CYAN = "\033[96m"
RESET = "\033[0m"


def get_tests_from_cmakelists():
    cmake_path = "CMakeLists.txt"
    if not os.path.exists(cmake_path):
        print(f"{RED}Error: {cmake_path} not found in current directory.{RESET}")
        sys.exit(1)

    with open(cmake_path, "r") as f:
        content = f.read()

    # Remove all commented lines to prevent running commented-out tests
    content = re.sub(r"#.*", "", content)

    # Use regex to find all files inside add_rostest(...)
    # It matches patterns like: add_rostest(test/my_test.launch)
    test_files = []
    matches = re.finditer(r"add_rostest\s*\(\s*([^\s\)]+)\s*\)", content)

    for match in matches:
        file_path = match.group(1)
        # Clean up common CMake variables if they exist
        file_path = file_path.replace("${PROJECT_SOURCE_DIR}/", "")
        file_path = file_path.replace("${CMAKE_CURRENT_SOURCE_DIR}/", "")

        if os.path.exists(file_path):
            test_files.append(file_path)
        else:
            print(
                f"{YELLOW}[Warning] Test defined in CMakeLists not found on disk: {file_path}{RESET}"
            )

    return test_files


def run_all_tests():
    print(f"{CYAN}Parsing CMakeLists.txt for rostest targets...{RESET}")
    test_files = get_tests_from_cmakelists()

    if not test_files:
        print(
            f"{YELLOW}No active add_rostest() targets found in CMakeLists.txt.{RESET}"
        )
        return

    total_tests = len(test_files)
    passed_count = 0
    failed_count = 0

    print(f"{CYAN}=================================================={RESET}")
    print(f"{CYAN} Found {total_tests} test(s). Starting execution...{RESET}")
    print(f"{CYAN}=================================================={RESET}\n")

    for index, launch_file in enumerate(test_files, start=1):
        print(
            f"{YELLOW}[Progress: {index}/{total_tests}] Running: {launch_file}{RESET}"
        )
        print(f"{YELLOW}--------------------------------------------------{RESET}")

        # Command to run rostest with real-time text output
        cmd = ["rostest", "--text", launch_file]

        # Execute command and stream output in real-time
        process = subprocess.Popen(
            cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            universal_newlines=True,
        )

        # Print output line by line
        for line in process.stdout:
            print(line, end="")

        process.wait()

        # Check execution result
        if process.returncode == 0:
            passed_count += 1
            print(f"\n{GREEN}[SUCCESS] Test {index} passed: {launch_file}{RESET}\n")
        else:
            failed_count += 1
            print(f"\n{RED}[FAILED] Test {index} failed: {launch_file}{RESET}\n")

    # Print final summary
    print(f"{CYAN}=================================================={RESET}")
    print(f"{CYAN} TEST SUMMARY{RESET}")
    print(f"{CYAN}=================================================={RESET}")
    print(f" Total : {total_tests}")
    print(f"{GREEN} Passed: {passed_count}{RESET}")
    if failed_count > 0:
        print(f"{RED} Failed: {failed_count}{RESET}")
        sys.exit(1)
    else:
        sys.exit(0)


if __name__ == "__main__":
    run_all_tests()
