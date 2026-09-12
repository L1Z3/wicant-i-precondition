param(
    [string]$SrcPath = "main/homepage_full.html",
    [string]$DstPath = "main/homepage.html"
)

if (-not (Test-Path $SrcPath)) {
    Write-Error "Source file $SrcPath not found."
    exit 1
}

$content = [System.IO.File]::ReadAllText((Resolve-Path $SrcPath), [System.Text.Encoding]::UTF8)

# 1. Remove HTML comments
$content = [System.Text.RegularExpressions.Regex]::Replace($content, "<!--(?!\[if)[\s\S]*?-->", "")

# 2. Minify embedded <style> blocks
$styleEvaluator = [System.Text.RegularExpressions.MatchEvaluator]{
    param($match)
    $css = $match.Groups[1].Value
    $css = [System.Text.RegularExpressions.Regex]::Replace($css, "/\*[\s\S]*?\*/", "")
    $css = [System.Text.RegularExpressions.Regex]::Replace($css, "\s+", " ")
    $css = [System.Text.RegularExpressions.Regex]::Replace($css, "\s*([\{\}:;,])\s*", '$1')
    return "<style>" + $css.Trim() + "</style>"
}
$content = [System.Text.RegularExpressions.Regex]::Replace($content, "<style[^>]*>([\s\S]*?)</style>", $styleEvaluator)

# 3. Minify embedded <script> blocks
$scriptEvaluator = [System.Text.RegularExpressions.MatchEvaluator]{
    param($match)
    $js = $match.Groups[1].Value
    $js = [System.Text.RegularExpressions.Regex]::Replace($js, "/\*[\s\S]*?\*/", "")
    $jsLines = $js -split "\r?\n"
    $filtered = [System.Collections.Generic.List[string]]::new()
    foreach ($l in $jsLines) {
        $t = $l.Trim()
        if ([string]::IsNullOrWhiteSpace($t)) { continue }
        if ($t.StartsWith("//")) { continue }
        $filtered.Add($t)
    }
    return "<script>`n" + ($filtered -join "`n") + "`n</script>"
}
$content = [System.Text.RegularExpressions.Regex]::Replace($content, "<script[^>]*>([\s\S]*?)</script>", $scriptEvaluator)

# 4. Collapse blank lines and trim
$allLines = $content -split "\r?\n"
$finalLines = [System.Collections.Generic.List[string]]::new()
foreach ($l in $allLines) {
    $t = $l.Trim()
    if (-not [string]::IsNullOrWhiteSpace($t)) {
        $finalLines.Add($t)
    }
}
$minified = $finalLines -join "`n"

$dstResolved = [System.IO.Path]::GetFullPath((Join-Path (Get-Location) $DstPath))
[System.IO.File]::WriteAllText($dstResolved, $minified, (New-Object System.Text.UTF8Encoding($false)))

# 5. Gzip compress
$gzPath = if ($dstResolved.EndsWith(".gz")) { $dstResolved } else { $dstResolved + ".gz" }
$rawBytes = [System.IO.File]::ReadAllBytes($dstResolved)
$outFileStream = [System.IO.File]::Create($gzPath)
$gzipStream = New-Object System.IO.Compression.GZipStream($outFileStream, [System.IO.Compression.CompressionLevel]::Optimal)
$gzipStream.Write($rawBytes, 0, $rawBytes.Length)
$gzipStream.Close()
$outFileStream.Close()

$origSz = (Get-Item (Resolve-Path $SrcPath)).Length
$minSz = (Get-Item $dstResolved).Length
$gzSz = (Get-Item $gzPath).Length
$saved = $origSz - $gzSz
$pct = [Math]::Round(($saved / $origSz) * 100, 1)

Write-Host "Minified & Gzipped $SrcPath -> $gzPath : $origSz B -> $gzSz B (Saved $saved B / $([Math]::Round($saved/1024, 1)) KB, -$pct%)" -ForegroundColor Green
