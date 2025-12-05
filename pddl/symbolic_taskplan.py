# read_plan.py
import subprocess, re, sys

domain, problem = "blocksworld_domain.pddl", "blocksworld_problem.pddl"
cmd = ["python3","-m","pyperplan","-s","astar","-H","hff",domain,problem]
p = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)

steps=[]
for line in p.stdout.splitlines():
    m = re.match(r"^\s*\d+:\s*\(([^)]+)\)", line.strip().lower())
    if m:
        steps.append(m.group(1))          # e.g. "pick-up r1 a"
print("Plan found:")

if not steps:
    print(p.stdout)                       # show output if no steps parsed
    sys.exit(1)

plan=[(s.split()[0], s.split()[1:]) for s in steps]
for i,(a,args) in enumerate(plan):
    print(f"{i}: {a} {args}")
