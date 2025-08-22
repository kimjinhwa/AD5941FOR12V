# Check if tag option is provided
$createTag = $false
if ($args -contains "-t") {
    $createTag = $true
}

$content = Get-Content .\Version.h  -Encoding utf8 -TotalCount 1
if ($content -match '#define VERSION "(.*?)".*\/\/(.*)')
{
    $version = $matches[1].Trim()    # "SBMS_1.0.0" 추출
    $comment = $matches[2].Trim()    # "Release Version 1.0.0" 추출
    
    echo "Version: $version"
    echo "Comment: $comment"
    
    # Always commit and push
    git add -A
    git commit -am $comment
    git push origin main

    # Create tag only if -t option is provided
    if ($createTag) {
        Write-Host "Creating tag: $version"
        git tag -a $version -m $comment
        git push origin $version
    } else {
        Write-Host "Tag creation skipped. Use -t option to create a tag."
    }
}
else
{
    echo "Version.h 파일을 찾을 수 없습니다."
}