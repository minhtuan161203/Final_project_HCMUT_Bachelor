param(
    [string]$SourceHtml = "chapter6_source.html",
    [string]$OutputDocx = "..\\CHUONG_6_FIRMWARE_SOFTWARE_DRAFT.docx"
)

$ErrorActionPreference = "Stop"

function Get-CleanText {
    param([System.Xml.XmlNode]$Node)

    if ($null -eq $Node) {
        return ""
    }

    $text = $Node.InnerText
    if ($null -eq $text) {
        return ""
    }

    $text = $text -replace "\s+", " "
    return $text.Trim()
}

function New-Paragraph {
    param(
        $Selection,
        [string]$Text,
        [double]$FontSize = 13.0,
        [bool]$Bold = $false,
        [bool]$Italic = $false,
        [int]$Alignment = 3,
        [double]$SpaceAfter = 6.0,
        [double]$SpaceBefore = 0.0,
        [double]$LeftIndent = 0.0,
        [double]$RightIndent = 0.0,
        [double]$FirstLineIndent = 0.0,
        [string]$FontName = "Times New Roman",
        [int]$FontColor = 0,
        [bool]$AllCaps = $false
    )

    if ([string]::IsNullOrWhiteSpace($Text)) {
        return
    }

    $Selection.Style = "Normal"
    $Selection.Font.Name = $FontName
    $Selection.Font.Size = $FontSize
    $Selection.Font.Bold = [int]$Bold
    $Selection.Font.Italic = [int]$Italic
    $Selection.Font.Color = $FontColor
    $Selection.Font.AllCaps = [int]$AllCaps
    $Selection.ParagraphFormat.Alignment = $Alignment
    $Selection.ParagraphFormat.SpaceAfter = $SpaceAfter
    $Selection.ParagraphFormat.SpaceBefore = $SpaceBefore
    $Selection.ParagraphFormat.LeftIndent = $LeftIndent
    $Selection.ParagraphFormat.RightIndent = $RightIndent
    $Selection.ParagraphFormat.FirstLineIndent = $FirstLineIndent
    $Selection.ParagraphFormat.LineSpacingRule = 0
    $Selection.TypeText($Text)
    $Selection.TypeParagraph()
}

function New-ListParagraph {
    param(
        $Selection,
        [string]$Prefix,
        [string]$Text
    )

    New-Paragraph `
        -Selection $Selection `
        -Text "$Prefix $Text" `
        -FontSize 13.0 `
        -Bold $false `
        -Italic $false `
        -Alignment 3 `
        -SpaceAfter 4.0 `
        -LeftIndent 28.0 `
        -FirstLineIndent -14.0
}

function Add-XmlTable {
    param(
        $Document,
        $Selection,
        [System.Xml.XmlNode]$TableNode
    )

    $rows = @()
    foreach ($tr in $TableNode.SelectNodes("./tr")) {
        $cells = @()
        foreach ($cell in $tr.ChildNodes) {
            if (($cell.Name -eq "th") -or ($cell.Name -eq "td")) {
                $cells += ,$cell
            }
        }
        if ($cells.Count -gt 0) {
            $rows += ,@($cells)
        }
    }

    if ($rows.Count -eq 0) {
        return
    }

    $headers = @()
    foreach ($headerCell in $rows[0]) {
        $headers += (Get-CleanText $headerCell)
    }

    for ($r = 1; $r -lt $rows.Count; $r++) {
        $currentRow = $rows[$r]
        if ($currentRow.Count -ge 2) {
            $left = Get-CleanText $currentRow[0]
            $right = Get-CleanText $currentRow[1]
            New-ListParagraph -Selection $Selection -Prefix "-" -Text "${left}: $right"
        } else {
            $flat = @()
            foreach ($cell in $currentRow) {
                $flat += (Get-CleanText $cell)
            }
            if ($flat.Count -gt 0) {
                New-ListParagraph -Selection $Selection -Prefix "-" -Text ($flat -join " | ")
            }
        }
    }

    $Selection.TypeParagraph()
}

$scriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$sourcePath = Join-Path $scriptDir $SourceHtml
$outputPath = [System.IO.Path]::GetFullPath((Join-Path $scriptDir $OutputDocx))
$logPath = Join-Path $scriptDir "build_chapter6.log"

Set-Content -Path $logPath -Value "build-start" -Encoding UTF8

[xml]$xml = Get-Content $sourcePath -Raw -Encoding UTF8

$word = $null
$doc = $null

try {
    $word = New-Object -ComObject Word.Application
    $word.Visible = $false
    $word.DisplayAlerts = 0

    $doc = $word.Documents.Add()
    $doc.PageSetup.PaperSize = 7
    $doc.PageSetup.TopMargin = 70
    $doc.PageSetup.BottomMargin = 62
    $doc.PageSetup.LeftMargin = 85
    $doc.PageSetup.RightMargin = 62

    $selection = $word.Selection
    $selection.Font.Name = "Times New Roman"
    $selection.Font.Size = 13

    foreach ($node in $xml.html.body.ChildNodes) {
        if ($node.NodeType -ne [System.Xml.XmlNodeType]::Element) {
            continue
        }

        Add-Content -Path $logPath -Value ("node=" + $node.Name)

        switch ($node.Name) {
            "div" {
                $className = $node.Attributes["class"].Value
                switch ($className) {
                    "title" {
                        New-Paragraph `
                            -Selection $selection `
                            -Text (Get-CleanText $node) `
                            -FontSize 20 `
                            -Bold $true `
                            -Italic $false `
                            -Alignment 1 `
                            -SpaceAfter 8 `
                            -SpaceBefore 4
                    }
                    "subtitle" {
                        New-Paragraph `
                            -Selection $selection `
                            -Text (Get-CleanText $node) `
                            -FontSize 11.5 `
                            -Bold $false `
                            -Italic $false `
                            -Alignment 1 `
                            -SpaceAfter 12 `
                            -FontColor 8421504
                    }
                    "lead" {
                        foreach ($p in $node.SelectNodes("./p")) {
                            New-Paragraph `
                                -Selection $selection `
                                -Text (Get-CleanText $p) `
                                -FontSize 13.0 `
                                -Bold $false `
                                -Italic $false `
                                -Alignment 3 `
                                -SpaceAfter 7 `
                                -LeftIndent 18 `
                                -RightIndent 6
                        }
                    }
                    "figure-note" {
                        New-Paragraph `
                            -Selection $selection `
                            -Text (Get-CleanText $node) `
                            -FontSize 11.5 `
                            -Bold $false `
                            -Italic $true `
                            -Alignment 1 `
                            -SpaceAfter 10 `
                            -SpaceBefore 2 `
                            -LeftIndent 10 `
                            -RightIndent 10 `
                            -FontColor 8421504
                    }
                    "caption" {
                        New-Paragraph `
                            -Selection $selection `
                            -Text (Get-CleanText $node) `
                            -FontSize 11.5 `
                            -Bold $false `
                            -Italic $true `
                            -Alignment 1 `
                            -SpaceAfter 8 `
                            -FontColor 8421504
                    }
                    default {
                        New-Paragraph -Selection $selection -Text (Get-CleanText $node)
                    }
                }
            }
            "h1" {
                New-Paragraph `
                    -Selection $selection `
                    -Text (Get-CleanText $node) `
                    -FontSize 16 `
                    -Bold $true `
                    -Italic $false `
                    -Alignment 0 `
                    -SpaceAfter 6 `
                    -SpaceBefore 10 `
                    -FontColor 6118741
            }
            "h2" {
                New-Paragraph `
                    -Selection $selection `
                    -Text (Get-CleanText $node) `
                    -FontSize 14 `
                    -Bold $true `
                    -Italic $false `
                    -Alignment 0 `
                    -SpaceAfter 4 `
                    -SpaceBefore 8 `
                    -FontColor 7629150
            }
            "h3" {
                New-Paragraph `
                    -Selection $selection `
                    -Text (Get-CleanText $node) `
                    -FontSize 13 `
                    -Bold $true `
                    -Italic $false `
                    -Alignment 0 `
                    -SpaceAfter 4 `
                    -SpaceBefore 6
            }
            "p" {
                $className = ""
                $text = Get-CleanText $node
                $alignment = 3
                if ($null -ne $node.Attributes["class"]) {
                    $className = $node.Attributes["class"].Value
                }
                if ($text -match "[A-Za-z0-9]+_[A-Za-z0-9_]+") {
                    $alignment = 0
                }
                if ($className -eq "equation") {
                    New-Paragraph `
                        -Selection $selection `
                        -Text $text `
                        -FontSize 12.5 `
                        -Bold $false `
                        -Italic $true `
                        -Alignment 1 `
                        -SpaceAfter 8
                } elseif ($className -eq "small") {
                    New-Paragraph `
                        -Selection $selection `
                        -Text $text `
                        -FontSize 11 `
                        -Bold $false `
                        -Italic $false `
                        -Alignment 3 `
                        -SpaceAfter 6 `
                        -FontColor 8421504
                } else {
                    New-Paragraph `
                        -Selection $selection `
                        -Text $text `
                        -FontSize 13 `
                        -Bold $false `
                        -Italic $false `
                        -Alignment $alignment `
                        -SpaceAfter 6
                }
            }
            "ul" {
                foreach ($li in $node.SelectNodes("./li")) {
                    New-ListParagraph -Selection $selection -Prefix "-" -Text (Get-CleanText $li)
                }
                $selection.TypeParagraph()
            }
            "ol" {
                $index = 1
                foreach ($li in $node.SelectNodes("./li")) {
                    New-ListParagraph -Selection $selection -Prefix "$index." -Text (Get-CleanText $li)
                    $index++
                }
                $selection.TypeParagraph()
            }
            "table" {
                Add-XmlTable -Document $doc -Selection $selection -TableNode $node
            }
            default {
                $text = Get-CleanText $node
                if (-not [string]::IsNullOrWhiteSpace($text)) {
                    New-Paragraph -Selection $selection -Text $text
                }
            }
        }
    }

    Add-Content -Path $logPath -Value "save-start"
    $doc.SaveAs2($outputPath, 16)
    Add-Content -Path $logPath -Value "save-done"
    Write-Output $outputPath
}
finally {
    Add-Content -Path $logPath -Value "finally"
    if ($null -ne $doc) {
        $doc.Close(0) | Out-Null
    }
    if ($null -ne $word) {
        $word.Quit() | Out-Null
    }
}
