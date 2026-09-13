#!/usr/bin/env python3
"""
Lightweight, dependency-free HTML/CSS/JS minifier for WiCAN firmware web assets.
Reduces raw HTML/CSS/JS size to optimize firmware binary flash usage.
"""

import sys
import os
import re

def minify_html(src_path, dst_path):
    if not os.path.exists(src_path):
        print(f"Error: Source file {src_path} not found.")
        sys.exit(1)

    with open(src_path, "r", encoding="utf-8") as f:
        content = f.read()

    # 1. Remove HTML comments
    content = re.sub(r'<!--(?!\[if).*?-->', '', content, flags=re.DOTALL)

    # 2. Minify embedded <style> blocks
    def minify_css(match):
        css = match.group(1)
        css = re.sub(r'/\*.*?\*/', '', css, flags=re.DOTALL)
        css = re.sub(r'\s+', ' ', css)
        css = re.sub(r'\s*([\{\}:;,])\s*', r'\1', css)
        return f'<style>{css.strip()}</style>'

    content = re.sub(r'<style[^>]*>(.*?)</style>', minify_css, content, flags=re.DOTALL)

    # 3. Minify embedded <script> blocks
    def minify_js(match):
        js = match.group(1)
        # Strip multi-line comments
        js = re.sub(r'/\*.*?\*/', '', js, flags=re.DOTALL)
        lines = []
        for line in js.splitlines():
            l = line.strip()
            if not l:
                continue
            # Strip pure single-line comment lines
            if l.startswith('//'):
                continue
            lines.append(l)
        return f'<script>\n' + '\n'.join(lines) + '\n</script>'

    content = re.sub(r'<script[^>]*>(.*?)</script>', minify_js, content, flags=re.DOTALL)

    # 4. Collapse consecutive blank lines and redundant whitespace in HTML markup
    lines = [line.strip() for line in content.splitlines() if line.strip()]
    minified = '\n'.join(lines)

    with open(dst_path, "w", encoding="utf-8", newline="\n") as f:
        f.write(minified)

    # 5. Gzip compress for firmware flash embedding
    gz_path = dst_path if dst_path.endswith(".gz") else (dst_path + ".gz")
    import gzip
    with open(dst_path, "rb") as f_in, gzip.open(gz_path, "wb", compresslevel=9) as f_out:
        f_out.write(f_in.read())

    orig_sz = os.path.getsize(src_path)
    min_sz = os.path.getsize(dst_path)
    gz_sz = os.path.getsize(gz_path)
    saved = orig_sz - gz_sz
    pct = (saved / orig_sz) * 100 if orig_sz > 0 else 0
    print(f"Minified & Gzipped {src_path} -> {gz_path}: {orig_sz:,} B -> {gz_sz:,} B (Saved {saved:,} B / {saved/1024:.1f} KB, -{pct:.1f}%)")

if __name__ == "__main__":
    src = sys.argv[1] if len(sys.argv) > 1 else "main/homepage_full.html"
    dst = sys.argv[2] if len(sys.argv) > 2 else "main/homepage.html"
    minify_html(src, dst)
