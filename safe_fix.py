import os
import re

def absolute_safe_fix(directory):
    files_fixed = 0

    for root, _, files in os.walk(directory):
        for file in files:
            if file.endswith((".md", ".mdx")):
                path = os.path.join(root, file)
                with open(path, 'r', encoding='utf-8') as f:
                    full_content = f.read()

                # 1. Protect Front Matter (Uses regex to find the start/end dashes)
                parts = re.split(r'^---$', full_content, maxsplit=2, flags=re.MULTILINE)
                
                if len(parts) == 3:
                    front_matter = parts[1]
                    body = parts[2]
                else:
                    front_matter = None
                    body = full_content

                def fix_mermaid_block(match):
                    type_line = match.group(1).strip()
                    content = match.group(2)
                    
                    # --- GANTT CHART LOGIC ---
                    if 'gantt' in type_line.lower():
                        # Remove illegal 'fill:#' from task lines
                        c = re.sub(r'fill\s*:\s*#[A-Fa-f0-9]+', '', content)
                        # Ensure space before colon for task data
                        c = re.sub(r'([^\s\n])\s*:', r'\1 :', c)
                        return f"```mermaid\n{type_line}\n{c}\n```"

                    # --- FLOWCHART LOGIC ---
                    # A. Clear messy existing quotes
                    c = re.sub(r'\["+(.*?)"*\]', r'[\1]', content)
                    
                    # B. Quote EVERYTHING inside brackets. 
                    # This is the most compatible way for Mermaid.
                    c = re.sub(r'([\w-]+)\[([^"\]\n\r]+?)\]', r'\1["\2"]', c)
                    
                    # C. Force spaces around arrows so they don't touch quotes
                    c = re.sub(r'(\"|\])(--+|--+>)(\"|\[|[\w-]+)', r'\1 \2 \3', c)
                    
                    # D. Fix subgraphs
                    c = re.sub(r'subgraph\s+([^"\n\r]+?[\s\(\)].*?)$', r'subgraph "\1"', c, flags=re.MULTILINE)
                    
                    return f"```mermaid\n{type_line}\n{c}\n```"

                # Apply to every mermaid block
                new_body = re.sub(r'```mermaid\s*\n(.*?)\n([\s\S]*?)\n```', fix_mermaid_block, body)

                if body != new_body:
                    if front_matter:
                        new_content = f"---{front_matter}---{new_body}"
                    else:
                        new_content = new_body
                        
                    with open(path, 'w', encoding='utf-8') as f:
                        f.write(new_content)
                    print(f"✅ Fixed: {path}")
                    files_fixed += 1

    print(f"\nFinished! Fixed {files_fixed} files.")

absolute_safe_fix('./docs')