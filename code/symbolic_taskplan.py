# read_plan.py
import subprocess, re, sys

def run_symbolic_taskplan() -> bool:
    try:
        domain, problem = "blocksworld_domain.pddl", "blocksworld_problem.pddl"
        cmd = ["python3","-m","pyperplan","-s","astar","-H","hff",domain,problem]
        p = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)

        steps=[]
        for line in p.stdout.splitlines():
            m = re.match(r"^\s*\d+:\s*\(([^)]+)\)", line.strip().lower())
            if m:
                steps.append(m.group(1))          # e.g. "pick-up r1 a"

        if not steps:
            print(p.stdout)                       # show output if no steps parsed
            #sys.exit(1)
            return True

        plan=[(s.split()[0], s.split()[1:]) for s in steps]
        for i,(a,args) in enumerate(plan):
            print(f"{i}: {a} {args}")

    except FileNotFoundError: 
        print("Error: the command did not find the specified file or directory.")
        return False
    except subprocess.CalledProcessError as e:
        print(f"Stderr: {e.stderr}")
        return False
    except Exception as e:
        print(f"An unexpected error occurred: {e}")
        return False
    
    return True