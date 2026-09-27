import sys, re, collections
# py-spy raw (collapsed) → 顶层函数包含时间占比
def load(p):
    rows = []
    for ln in open(p, encoding="utf-8", errors="replace"):
        ln = ln.rstrip("\n")
        if not ln: continue
        stack, _, n = ln.rpartition(" ")
        try: rows.append((stack.split(";"), int(n)))
        except ValueError: pass
    return rows
def fn(f):
    m = re.match(r"(\S+) \(([^:]+)(?::\d+)?\)", f)
    return f"{m.group(1)} ({m.group(2).split('/')[-1]})" if m else f
for p in sys.argv[1:]:
    rows = load(p); tot = sum(n for _, n in rows) or 1
    inc, self_ = collections.Counter(), collections.Counter()
    for st, n in rows:
        seen = set()
        for f in st[1:]:
            k = fn(f)
            if k not in seen: inc[k] += n; seen.add(k)
        self_[fn(st[-1])] += n
    print(f"== {p}  samples={tot}")
    print(" 包含 (inclusive):")
    for k, n in inc.most_common(28): print(f"  {100*n/tot:5.1f}%  {k}")
    print(" 自身 (self):")
    for k, n in self_.most_common(12): print(f"  {100*n/tot:5.1f}%  {k}")
