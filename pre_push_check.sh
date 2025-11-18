#!/bin/bash

echo "=== VERIFICAÇÃO PRÉ-PUSH ==="
echo

echo "1. Configuração do Git:"
git config user.name
git config user.email
echo

echo "2. Branch atual:"
git branch --show-current
echo

echo "3. Status do repositório:"
git status --porcelain
if [ $? -eq 0 ] && [ -z "$(git status --porcelain)" ]; then
    echo "✅ Working tree limpo"
else
    echo "⚠️  Existem mudanças não commitadas"
fi
echo

echo "4. Últimos commits:"
git log --oneline -5
echo

echo "5. Remotes configurados:"
git remote -v
echo

echo "6. Arquivos que serão enviados (amostra):"
echo "📁 Arquivos principais:"
ls -la | grep -E "\.(md|py|txt|yml|yaml|launch|urdf|xacro)$" | head -10
echo

echo "7. Tamanho aproximado do repositório:"
du -sh .git
echo

echo "8. Verificando se há arquivos grandes (>10MB):"
find . -type f -size +10M -not -path "./.git/*" -not -path "./build/*" -not -path "./install/*" -not -path "./log/*" 2>/dev/null || echo "Nenhum arquivo grande encontrado"
echo

echo "=== VERIFICAÇÃO CONCLUÍDA ==="
echo "Se tudo estiver correto, você pode fazer o push com:"
echo "git push -u origin master"
