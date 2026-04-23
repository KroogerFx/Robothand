param(
    [string]$Port,
    [int]$BaudRate = 115200,
    [ValidateSet("none", "lf", "cr", "crlf")]
    [string]$LineEnding = "lf",
    [switch]$Cli,
    [switch]$Help
)

$ErrorActionPreference = "Stop"

Add-Type -AssemblyName System.Windows.Forms
Add-Type -AssemblyName System.Drawing

$script:lineEndingMap = @{
    none = ""
    lf   = "`n"
    cr   = "`r"
    crlf = "`r`n"
}

function Show-Usage {
    Write-Host "Robot Hand Serial Terminal"
    Write-Host ""
    Write-Host "Usage:"
    Write-Host "  .\run_terminal.cmd"
    Write-Host "  .\run_terminal.cmd COM3"
    Write-Host "  .\run_terminal.cmd -BaudRate 115200 -LineEnding lf"
    Write-Host "  .\run_terminal.cmd -Cli"
    Write-Host ""
    Write-Host "CLI local commands:"
    Write-Host "  :help"
    Write-Host "  :ports"
    Write-Host "  :open"
    Write-Host "  :open COM3"
    Write-Host "  :close"
    Write-Host "  :baud 115200"
    Write-Host "  :ending lf"
    Write-Host "  :quit"
}

function Get-PortDescriptions {
    $map = @{}

    try {
        $devices = Get-CimInstance Win32_PnPEntity -ErrorAction Stop |
            Where-Object { $_.Name -match '\((COM\d+)\)' }

        foreach ($device in $devices) {
            $portName = $matches[1]
            if (-not $map.ContainsKey($portName)) {
                $map[$portName] = $device.Name
            }
        }
    } catch {
    }

    return $map
}

function Get-Ports {
    $descriptions = Get-PortDescriptions
    $ports = [System.IO.Ports.SerialPort]::GetPortNames() | Sort-Object

    foreach ($portName in $ports) {
        [pscustomobject]@{
            Device = $portName
            Description = if ($descriptions.ContainsKey($portName)) { $descriptions[$portName] } else { "Serial Port" }
        }
    }
}

function Resolve-PortChoice([string]$Choice, $Ports) {
    if ([string]::IsNullOrWhiteSpace($Choice)) {
        return $null
    }

    if ($Choice -match '^\d+$') {
        $index = [int]$Choice - 1
        if ($index -ge 0 -and $index -lt $Ports.Count) {
            return $Ports[$index].Device
        }
        return $null
    }

    foreach ($port in $Ports) {
        if ($port.Device -ieq $Choice) {
            return $port.Device
        }
    }

    return $null
}

function Start-CliTerminal {
    $script:serialPort = $null
    $script:baudRate = $BaudRate
    $script:lineEnding = $LineEnding
    $inputBuffer = New-Object System.Text.StringBuilder

    function Show-Ports {
        $ports = @(Get-Ports)
        if ($ports.Count -eq 0) {
            Write-Host "No serial ports found."
            return $ports
        }

        Write-Host "Available ports:"
        for ($index = 0; $index -lt $ports.Count; $index++) {
            $port = $ports[$index]
            Write-Host ("  {0}. {1} - {2}" -f ($index + 1), $port.Device, $port.Description)
        }

        return $ports
    }

    function Close-Port {
        if ($script:serialPort) {
            try {
                if ($script:serialPort.IsOpen) {
                    $script:serialPort.Close()
                }
            } catch {
            } finally {
                $script:serialPort.Dispose()
                $script:serialPort = $null
            }
        }
    }

    function Open-Port([string]$PortName) {
        Close-Port

        try {
            $port = [System.IO.Ports.SerialPort]::new($PortName, $script:baudRate)
            $port.Encoding = [System.Text.Encoding]::UTF8
            $port.NewLine = $script:lineEndingMap[$script:lineEnding]
            $port.ReadTimeout = 25
            $port.WriteTimeout = 1000
            $port.Open()
        } catch {
            Write-Host ("Failed to open {0}: {1}" -f $PortName, $_.Exception.Message)
            return
        }

        $script:serialPort = $port
        Write-Host ("Connected to {0} at {1} baud." -f $PortName, $script:baudRate)
    }

    function Write-Prompt([string]$CurrentText = "") {
        [Console]::Write(("`r> {0}" -f $CurrentText))
        $width = [Math]::Max([Console]::WindowWidth - ($CurrentText.Length + 2), 0)
        if ($width -gt 0) {
            [Console]::Write((" " * $width))
            [Console]::Write(("`r> {0}" -f $CurrentText))
        }
    }

    function Show-ReceivedText([string]$Text) {
        [Console]::Write("`r")
        [Console]::Write((" " * [Math]::Max([Console]::WindowWidth - 1, 0)))
        [Console]::Write("`r")
        [Console]::Write(("[RX] {0}" -f $Text))
        if (-not $Text.EndsWith("`n")) {
            [Console]::Write("`r`n")
        }
        Write-Prompt $inputBuffer.ToString()
    }

    function Receive-SerialInput {
        if (-not $script:serialPort -or -not $script:serialPort.IsOpen) {
            return
        }

        try {
            $text = $script:serialPort.ReadExisting()
            if (-not [string]::IsNullOrEmpty($text)) {
                Show-ReceivedText $text
            }
        } catch {
            Write-Host ""
            Write-Host ("Serial read stopped: {0}" -f $_.Exception.Message)
            Close-Port
            Write-Prompt $inputBuffer.ToString()
        }
    }

    function Select-Port([bool]$InitialPrompt = $false) {
        $ports = @(Show-Ports)
        if ($ports.Count -eq 0) {
            if ($InitialPrompt) {
                Write-Host "Plug in the ESP32 and use :ports or :open once it appears."
            }
            return
        }

        $current = if ($script:serialPort -and $script:serialPort.IsOpen) { $script:serialPort.PortName } else { $null }
        $prompt = "Select a port number"
        if ($current) {
            $prompt += " or press Enter to keep $current"
        }
        $prompt += ": "

        $selection = Read-Host $prompt
        if ([string]::IsNullOrWhiteSpace($selection)) {
            if ($current) {
                Write-Host "Keeping current port $current."
            } else {
                Write-Host "No port selected."
            }
            return
        }

        $resolved = Resolve-PortChoice $selection $ports
        if (-not $resolved) {
            Write-Host "Could not match port selection: $selection"
            return
        }

        Open-Port $resolved
    }

    function Set-BaudRate([string]$Value) {
        $parsed = 0
        if (-not [int]::TryParse($Value, [ref]$parsed)) {
            Write-Host "Invalid baud rate: $Value"
            return
        }

        $reconnectPort = if ($script:serialPort -and $script:serialPort.IsOpen) { $script:serialPort.PortName } else { $null }
        $script:baudRate = $parsed
        Write-Host "Baud rate set to $parsed."

        if ($reconnectPort) {
            Write-Host "Reconnecting to $reconnectPort..."
            Open-Port $reconnectPort
        }
    }

    function Set-LineEnding([string]$Value) {
        $option = $Value.ToLowerInvariant()
        if (-not $script:lineEndingMap.ContainsKey($option)) {
            Write-Host "Invalid line ending: $Value"
            Write-Host "Choose one of: none, lf, cr, or crlf"
            return
        }

        $script:lineEnding = $option
        Write-Host "Line ending set to $option."

        if ($script:serialPort -and $script:serialPort.IsOpen) {
            $script:serialPort.NewLine = $script:lineEndingMap[$option]
        }
    }

    function Send-Line([string]$Text) {
        if (-not $script:serialPort -or -not $script:serialPort.IsOpen) {
            Write-Host "No serial port is open. Use :open to connect first."
            return
        }

        try {
            $payload = $Text + $script:lineEndingMap[$script:lineEnding]
            $script:serialPort.Write($payload)
        } catch {
            Write-Host ("Send failed: {0}" -f $_.Exception.Message)
            Close-Port
        }
    }

    function Invoke-LocalCommand([string]$CommandLine) {
        $parts = @($CommandLine -split '\s+' | Where-Object { $_ -ne "" })
        if ($parts.Count -eq 0) {
            return $true
        }

        $command = $parts[0].ToLowerInvariant()
        $commandArgs = if ($parts.Count -gt 1) { $parts[1..($parts.Count - 1)] } else { @() }

        switch ($command) {
            "help" {
                Show-Usage
                return $true
            }
            "ports" {
                [void](Show-Ports)
                return $true
            }
            "open" {
                if ($commandArgs.Count -gt 0) {
                    $ports = @(Get-Ports)
                    $resolved = Resolve-PortChoice $commandArgs[0] $ports
                    if (-not $resolved) {
                        Write-Host "Could not match port selection: $($commandArgs[0])"
                        return $true
                    }
                    Open-Port $resolved
                } else {
                    Select-Port
                }
                return $true
            }
            "close" {
                Close-Port
                Write-Host "Port closed."
                return $true
            }
            "baud" {
                if ($commandArgs.Count -eq 0) {
                    Write-Host "Current baud rate: $script:baudRate"
                } else {
                    Set-BaudRate $commandArgs[0]
                }
                return $true
            }
            "ending" {
                if ($commandArgs.Count -eq 0) {
                    Write-Host "Current line ending: $script:lineEnding"
                } else {
                    Set-LineEnding $commandArgs[0]
                }
                return $true
            }
            "quit" {
                return $false
            }
            default {
                Write-Host "Unknown local command: $command"
                Write-Host "Use :help to see available commands."
                return $true
            }
        }
    }

    Write-Host "Robot Hand Serial Terminal"
    Write-Host "Type text and press Enter to send it."
    Write-Host "Local commands: :help, :ports, :open, :close, :baud, :ending, :quit"
    Write-Host ""

    if ($Port) {
        Open-Port $Port
    } else {
        Select-Port $true
    }

    try {
        Write-Prompt
        while ($true) {
            Receive-SerialInput

            if ([Console]::KeyAvailable) {
                $key = [Console]::ReadKey($true)

                switch ($key.Key) {
                    ([ConsoleKey]::Enter) {
                        $userInput = $inputBuffer.ToString()
                        $inputBuffer.Clear() | Out-Null
                        [Console]::Write("`r`n")

                        if ([string]::IsNullOrWhiteSpace($userInput)) {
                            Write-Prompt
                            continue
                        }

                        if ($userInput.StartsWith(":")) {
                            if (-not (Invoke-LocalCommand $userInput.Substring(1))) {
                                break
                            }
                            Write-Prompt
                            continue
                        }

                        Send-Line $userInput
                        Write-Prompt
                        continue
                    }
                    ([ConsoleKey]::Backspace) {
                        if ($inputBuffer.Length -gt 0) {
                            $inputBuffer.Remove($inputBuffer.Length - 1, 1) | Out-Null
                            Write-Prompt $inputBuffer.ToString()
                        }
                        continue
                    }
                }

                if (-not [char]::IsControl($key.KeyChar)) {
                    [void]$inputBuffer.Append($key.KeyChar)
                    Write-Prompt $inputBuffer.ToString()
                }
            }

            Start-Sleep -Milliseconds 25
        }
    } finally {
        Write-Host ""
        Close-Port
        Write-Host "Closing terminal."
    }
}

function Start-GuiTerminal {
    . (Join-Path $PSScriptRoot "gui_terminal.ps1")
    Start-RobotHandGui -Port $Port -BaudRate $BaudRate -LineEnding $LineEnding -LineEndingMap $script:lineEndingMap -GetPortsBlock ${function:Get-Ports}
    return

    [System.Windows.Forms.Application]::EnableVisualStyles()

    $lineEndingMap = if ($script:lineEndingMap) {
        $script:lineEndingMap
    } else {
        @{
            none = ""
            lf   = "`n"
            cr   = "`r"
            crlf = "`r`n"
        }
    }

    $form = New-Object System.Windows.Forms.Form
    $form.Text = "Robot Hand Serial Terminal"
    $form.StartPosition = "CenterScreen"
    $form.Size = New-Object System.Drawing.Size(1320, 820)
    $form.MinimumSize = New-Object System.Drawing.Size(1120, 700)

    $state = @{
        SerialPort = $null
        PollTimer = $null
    }

    $topPanel = New-Object System.Windows.Forms.Panel
    $topPanel.Dock = "Top"
    $topPanel.Height = 58
    $topPanel.Padding = New-Object System.Windows.Forms.Padding(12)
    $form.Controls.Add($topPanel)

    $statusLabel = New-Object System.Windows.Forms.Label
    $statusLabel.Dock = "Bottom"
    $statusLabel.Height = 24
    $statusLabel.Padding = New-Object System.Windows.Forms.Padding(8, 4, 8, 4)
    $statusLabel.BorderStyle = "Fixed3D"
    $statusLabel.Text = "Disconnected"
    $form.Controls.Add($statusLabel)

    $bottomPanel = New-Object System.Windows.Forms.Panel
    $bottomPanel.Dock = "Bottom"
    $bottomPanel.Height = 96
    $bottomPanel.Padding = New-Object System.Windows.Forms.Padding(12)
    $form.Controls.Add($bottomPanel)

    $commandPanel = New-Object System.Windows.Forms.Panel
    $commandPanel.Dock = "Right"
    $commandPanel.Width = 370
    $commandPanel.Padding = New-Object System.Windows.Forms.Padding(12, 12, 12, 0)
    $form.Controls.Add($commandPanel)

    $mainPanel = New-Object System.Windows.Forms.Panel
    $mainPanel.Dock = "Fill"
    $mainPanel.Padding = New-Object System.Windows.Forms.Padding(12, 12, 0, 0)
    $form.Controls.Add($mainPanel)

    $logSplit = New-Object System.Windows.Forms.SplitContainer
    $logSplit.Dock = "Fill"
    $logSplit.Orientation = "Horizontal"
    $mainPanel.Controls.Add($logSplit)

    $rxGroup = New-Object System.Windows.Forms.GroupBox
    $rxGroup.Dock = "Fill"
    $rxGroup.Text = "Incoming Data (RX)"
    $logSplit.Panel1.Controls.Add($rxGroup)

    $rxLogBox = New-Object System.Windows.Forms.RichTextBox
    $rxLogBox.Dock = "Fill"
    $rxLogBox.ReadOnly = $true
    $rxLogBox.BackColor = [System.Drawing.Color]::White
    $rxLogBox.Font = New-Object System.Drawing.Font("Consolas", 10)
    $rxLogBox.HideSelection = $false
    $rxGroup.Controls.Add($rxLogBox)

    $txGroup = New-Object System.Windows.Forms.GroupBox
    $txGroup.Dock = "Fill"
    $txGroup.Text = "Outgoing Data / System (TX)"
    $logSplit.Panel2.Controls.Add($txGroup)

    $txLogBox = New-Object System.Windows.Forms.RichTextBox
    $txLogBox.Dock = "Fill"
    $txLogBox.ReadOnly = $true
    $txLogBox.BackColor = [System.Drawing.Color]::White
    $txLogBox.Font = New-Object System.Drawing.Font("Consolas", 10)
    $txLogBox.HideSelection = $false
    $txGroup.Controls.Add($txLogBox)

    $commandTabs = New-Object System.Windows.Forms.TabControl
    $commandTabs.Dock = "Fill"
    $commandPanel.Controls.Add($commandTabs)

    $quickTab = New-Object System.Windows.Forms.TabPage
    $quickTab.Text = "Quick Commands"
    [void]$commandTabs.TabPages.Add($quickTab)

    $buildersTab = New-Object System.Windows.Forms.TabPage
    $buildersTab.Text = "Command Builders"
    [void]$commandTabs.TabPages.Add($buildersTab)

    $quickLayout = New-Object System.Windows.Forms.FlowLayoutPanel
    $quickLayout.Dock = "Fill"
    $quickLayout.FlowDirection = "LeftToRight"
    $quickLayout.WrapContents = $true
    $quickLayout.AutoScroll = $true
    $quickLayout.Padding = New-Object System.Windows.Forms.Padding(8)
    $quickTab.Controls.Add($quickLayout)

    $buildersFlow = New-Object System.Windows.Forms.FlowLayoutPanel
    $buildersFlow.Dock = "Fill"
    $buildersFlow.FlowDirection = "TopDown"
    $buildersFlow.WrapContents = $false
    $buildersFlow.AutoScroll = $true
    $buildersFlow.Padding = New-Object System.Windows.Forms.Padding(8)
    $buildersTab.Controls.Add($buildersFlow)

    $portLabel = New-Object System.Windows.Forms.Label
    $portLabel.Text = "Port"
    $portLabel.Location = New-Object System.Drawing.Point(12, 18)
    $portLabel.AutoSize = $true
    $topPanel.Controls.Add($portLabel)

    $portCombo = New-Object System.Windows.Forms.ComboBox
    $portCombo.Location = New-Object System.Drawing.Point(48, 14)
    $portCombo.Size = New-Object System.Drawing.Size(320, 24)
    $portCombo.DropDownStyle = "DropDownList"
    $topPanel.Controls.Add($portCombo)

    $refreshButton = New-Object System.Windows.Forms.Button
    $refreshButton.Text = "Refresh"
    $refreshButton.Location = New-Object System.Drawing.Point(380, 12)
    $refreshButton.Size = New-Object System.Drawing.Size(78, 28)
    $topPanel.Controls.Add($refreshButton)

    $connectButton = New-Object System.Windows.Forms.Button
    $connectButton.Text = "Connect"
    $connectButton.Location = New-Object System.Drawing.Point(466, 12)
    $connectButton.Size = New-Object System.Drawing.Size(88, 28)
    $topPanel.Controls.Add($connectButton)

    $baudLabel = New-Object System.Windows.Forms.Label
    $baudLabel.Text = "Baud"
    $baudLabel.Location = New-Object System.Drawing.Point(568, 18)
    $baudLabel.AutoSize = $true
    $topPanel.Controls.Add($baudLabel)

    $baudCombo = New-Object System.Windows.Forms.ComboBox
    $baudCombo.Location = New-Object System.Drawing.Point(608, 14)
    $baudCombo.Size = New-Object System.Drawing.Size(96, 24)
    $baudCombo.DropDownStyle = "DropDown"
    [void]$baudCombo.Items.AddRange(@("9600", "57600", "115200", "230400"))
    $baudCombo.Text = [string]$BaudRate
    $topPanel.Controls.Add($baudCombo)

    $endingLabel = New-Object System.Windows.Forms.Label
    $endingLabel.Text = "Line Ending"
    $endingLabel.Location = New-Object System.Drawing.Point(720, 18)
    $endingLabel.AutoSize = $true
    $topPanel.Controls.Add($endingLabel)

    $endingCombo = New-Object System.Windows.Forms.ComboBox
    $endingCombo.Location = New-Object System.Drawing.Point(798, 14)
    $endingCombo.Size = New-Object System.Drawing.Size(96, 24)
    $endingCombo.DropDownStyle = "DropDownList"
    [void]$endingCombo.Items.AddRange(@("none", "lf", "cr", "crlf"))
    $endingCombo.SelectedItem = $LineEnding
    if ($null -eq $endingCombo.SelectedItem) {
        $defaultEndingIndex = [Math]::Max(0, @("none", "lf", "cr", "crlf").IndexOf($LineEnding))
        $endingCombo.SelectedIndex = $defaultEndingIndex
    }
    $topPanel.Controls.Add($endingCombo)

    $inputBox = New-Object System.Windows.Forms.TextBox
    $inputBox.Location = New-Object System.Drawing.Point(12, 18)
    $inputBox.Size = New-Object System.Drawing.Size(860, 24)
    $bottomPanel.Controls.Add($inputBox)

    $sendButton = New-Object System.Windows.Forms.Button
    $sendButton.Text = "Send"
    $sendButton.Location = New-Object System.Drawing.Point(884, 16)
    $sendButton.Size = New-Object System.Drawing.Size(78, 28)
    $bottomPanel.Controls.Add($sendButton)

    $sendSelectedButton = New-Object System.Windows.Forms.Button
    $sendSelectedButton.Text = "Send Selected"
    $sendSelectedButton.Location = New-Object System.Drawing.Point(972, 16)
    $sendSelectedButton.Size = New-Object System.Drawing.Size(104, 28)
    $bottomPanel.Controls.Add($sendSelectedButton)

    $clearRxButton = New-Object System.Windows.Forms.Button
    $clearRxButton.Text = "Clear RX"
    $clearRxButton.Location = New-Object System.Drawing.Point(1088, 16)
    $clearRxButton.Size = New-Object System.Drawing.Size(82, 28)
    $bottomPanel.Controls.Add($clearRxButton)

    $clearTxButton = New-Object System.Windows.Forms.Button
    $clearTxButton.Text = "Clear TX"
    $clearTxButton.Location = New-Object System.Drawing.Point(1180, 16)
    $clearTxButton.Size = New-Object System.Drawing.Size(82, 28)
    $bottomPanel.Controls.Add($clearTxButton)

    $hintLabel = New-Object System.Windows.Forms.Label
    $hintLabel.Location = New-Object System.Drawing.Point(12, 54)
    $hintLabel.Size = New-Object System.Drawing.Size(1240, 24)
    $hintLabel.Text = "Type any raw command below, or use the quick buttons and builders on the right for calibrated presets and parameterized commands."
    $bottomPanel.Controls.Add($hintLabel)

    $portMap = @{}
    $script:RobotHandGui = [pscustomobject]@{
        InputBox = $inputBox
        ConnectButton = $connectButton
        StatusLabel = $statusLabel
        RxLogBox = $rxLogBox
        TxLogBox = $txLogBox
    }

    function Set-Status([string]$Message) {
        $script:RobotHandGui.StatusLabel.Text = $Message
    }

    function Write-BoxLog($TargetBox, [string]$Tag, [string]$Message) {
        $timestamp = Get-Date -Format "HH:mm:ss"
        $TargetBox.AppendText(("[{0}] [{1}] {2}" -f $timestamp, $Tag, $Message))
        if (-not $Message.EndsWith("`n")) {
            $TargetBox.AppendText("`r`n")
        }
        $TargetBox.SelectionStart = $TargetBox.TextLength
        $TargetBox.ScrollToCaret()
    }

    function Write-RxLog([string]$Tag, [string]$Message) {
        Write-BoxLog $rxLogBox $Tag $Message
    }

    function Write-TxLog([string]$Tag, [string]$Message) {
        Write-BoxLog $txLogBox $Tag $Message
    }

    function Write-SystemLog([string]$Message) {
        Write-TxLog "SYS" $Message
    }

    function New-CommandButton([string]$Label, [string]$CommandText, [int]$Width = 96) {
        $button = New-Object System.Windows.Forms.Button
        $button.Text = $Label
        $button.Width = $Width
        $button.Height = 34
        $button.Margin = New-Object System.Windows.Forms.Padding(6)
        $button.Add_Click({
            & $sendLineAction $CommandText
        }.GetNewClosure())
        return $button
    }

    function New-LabeledTextBox([string]$Label, [string]$DefaultValue, [int]$Left, [int]$Top, [int]$Width = 60) {
        $fieldLabel = New-Object System.Windows.Forms.Label
        $fieldLabel.Text = $Label
        $fieldLabel.Location = New-Object System.Drawing.Point -ArgumentList $Left, ($Top + 4)
        $fieldLabel.AutoSize = $true

        $fieldTextBox = New-Object System.Windows.Forms.TextBox
        $fieldTextBox.Location = New-Object System.Drawing.Point -ArgumentList ($Left + 76), $Top
        $fieldTextBox.Size = New-Object System.Drawing.Size -ArgumentList $Width, 24
        $fieldTextBox.Text = $DefaultValue

        return @{
            Label = $fieldLabel
            TextBox = $fieldTextBox
        }
    }

    function Add-BuilderGroup([string]$Title, [int]$Height) {
        $group = New-Object System.Windows.Forms.GroupBox
        $group.Text = $Title
        $group.Width = 320
        $group.Height = $Height
        $group.Margin = New-Object System.Windows.Forms.Padding(3, 3, 3, 10)
        $buildersFlow.Controls.Add($group)
        return $group
    }

    function Disconnect-Port {
        if ($state.PollTimer) {
            $state.PollTimer.Stop()
        }

        if ($state.SerialPort) {
            try {
                if ($state.SerialPort.IsOpen) {
                    $state.SerialPort.Close()
                }
            } catch {
            } finally {
                $state.SerialPort.Dispose()
                $state.SerialPort = $null
            }
        }

        $connectButton.Text = "Connect"
        Set-Status "Disconnected"
    }

    function Update-Ports {
        $currentPort = if ($state.SerialPort -and $state.SerialPort.IsOpen) { $state.SerialPort.PortName } else { $Port }
        $ports = @(Get-Ports)
        $portCombo.Items.Clear()
        $portMap.Clear()

        foreach ($entry in $ports) {
            $label = "{0} - {1}" -f $entry.Device, $entry.Description
            [void]$portCombo.Items.Add($label)
            $portMap[$label] = $entry.Device
        }

        if ($ports.Count -eq 0) {
            $portCombo.Text = ""
            Set-Status "No serial ports found"
            return
        }

        $preferred = $null
        if ($currentPort) {
            foreach ($label in $portMap.Keys) {
                if ($portMap[$label] -eq $currentPort) {
                    $preferred = $label
                    break
                }
            }
        }

        if (-not $preferred) {
            $preferred = [string]$portCombo.Items[0]
        }

        $portCombo.SelectedItem = $preferred
        Set-Status ("Found {0} serial port(s)" -f $ports.Count)
    }

    function Get-SelectedPort {
        $selection = [string]$portCombo.SelectedItem
        if ([string]::IsNullOrWhiteSpace($selection)) {
            $selection = $portCombo.Text
        }
        if ([string]::IsNullOrWhiteSpace($selection)) {
            return $null
        }
        if ($portMap.ContainsKey($selection)) {
            return $portMap[$selection]
        }
        return ($selection -split ' - ', 2)[0]
    }

    function Connect-Port {
        $portName = Get-SelectedPort
        if (-not $portName) {
            Write-SystemLog "Select a serial port first."
            Set-Status "No port selected"
            return
        }

        $parsedBaud = 0
        if (-not [int]::TryParse($baudCombo.Text, [ref]$parsedBaud)) {
            Write-SystemLog ("Invalid baud rate: {0}" -f $baudCombo.Text)
            Set-Status "Invalid baud rate"
            return
        }

        Disconnect-Port

        try {
            $newPort = [System.IO.Ports.SerialPort]::new($portName, $parsedBaud)
            $newPort.Encoding = [System.Text.Encoding]::UTF8
            $newPort.NewLine = $script:lineEndingMap[[string]$endingCombo.SelectedItem]
            $newPort.ReadTimeout = 25
            $newPort.WriteTimeout = 1000
            $newPort.Open()
            $state.SerialPort = $newPort

            if (-not $state.PollTimer) {
                $state.PollTimer = New-Object System.Windows.Forms.Timer
                $state.PollTimer.Interval = 50
                $state.PollTimer.Add_Tick({
                    if (-not $state.SerialPort -or -not $state.SerialPort.IsOpen) {
                        return
                    }

                    try {
                        $text = $state.SerialPort.ReadExisting()
                        if (-not [string]::IsNullOrEmpty($text)) {
                            Write-RxLog "RX" $text
                        }
                    } catch {
                        Write-SystemLog ("Serial read stopped: {0}" -f $_.Exception.Message)
                        Disconnect-Port
                    }
                }.GetNewClosure())
            }

            $state.PollTimer.Start()

            $connectButton.Text = "Disconnect"
            Write-SystemLog ("Connected to {0} at {1} baud" -f $portName, $parsedBaud)
            Set-Status ("Connected to {0}" -f $portName)
            $inputBox.Focus()
        } catch {
            Write-SystemLog ("Failed to open {0}: {1}" -f $portName, $_.Exception.Message)
            Set-Status ("Failed to open {0}" -f $portName)
        }
    }

    function Send-Line([string]$Text = $inputBox.Text, [switch]$PreserveInput) {
        $text = if ($null -eq $Text) { "" } else { $Text.Trim() }
        if ([string]::IsNullOrEmpty($text)) {
            return
        }

        if (-not $state.SerialPort -or -not $state.SerialPort.IsOpen) {
            Write-SystemLog "No serial port is open."
            Set-Status "No serial port is open"
            return
        }

        try {
            $payload = $text + $script:lineEndingMap[[string]$endingCombo.SelectedItem]
            $state.SerialPort.Write($payload)
            Write-TxLog "TX" $text
            if (-not $PreserveInput) {
                $inputBox.Clear()
            }
        } catch {
            Write-SystemLog ("Send failed: {0}" -f $_.Exception.Message)
            Disconnect-Port
        }
    }

    $setStatusAction = {
        param([string]$Message)
        $statusLabel.Text = $Message
    }.GetNewClosure()

    $appendBoxLogAction = {
        param($TargetBox, [string]$Tag, [string]$Message)
        $timestamp = Get-Date -Format "HH:mm:ss"
        $TargetBox.AppendText(("[{0}] [{1}] {2}" -f $timestamp, $Tag, $Message))
        if (-not $Message.EndsWith("`n")) {
            $TargetBox.AppendText("`r`n")
        }
        $TargetBox.SelectionStart = $TargetBox.TextLength
        $TargetBox.ScrollToCaret()
    }.GetNewClosure()

    $appendRxLogAction = {
        param([string]$Tag, [string]$Message)
        & $appendBoxLogAction $script:RobotHandGui.RxLogBox $Tag $Message
    }.GetNewClosure()

    $appendTxLogAction = {
        param([string]$Tag, [string]$Message)
        & $appendBoxLogAction $script:RobotHandGui.TxLogBox $Tag $Message
    }.GetNewClosure()

    $appendSystemLogAction = {
        param([string]$Message)
        & $appendTxLogAction "SYS" $Message
    }.GetNewClosure()

    $getSelectedPortAction = {
        $selection = [string]$portCombo.SelectedItem
        if ([string]::IsNullOrWhiteSpace($selection)) {
            $selection = $portCombo.Text
        }
        if ([string]::IsNullOrWhiteSpace($selection)) {
            return $null
        }
        if ($portMap.ContainsKey($selection)) {
            return $portMap[$selection]
        }
        return ($selection -split ' - ', 2)[0]
    }.GetNewClosure()

    $getLineEndingAction = {
        $selectedEnding = [string]$endingCombo.SelectedItem
        if ([string]::IsNullOrWhiteSpace($selectedEnding)) {
            $selectedEnding = $endingCombo.Text
        }
        if ([string]::IsNullOrWhiteSpace($selectedEnding) -or -not $lineEndingMap.ContainsKey($selectedEnding)) {
            $selectedEnding = if ($lineEndingMap.ContainsKey($LineEnding)) { $LineEnding } else { "lf" }
        }
        return $selectedEnding
    }.GetNewClosure()

    $disconnectPortAction = {
        if ($state.PollTimer) {
            $state.PollTimer.Stop()
        }

        if ($state.SerialPort) {
            try {
                if ($state.SerialPort.IsOpen) {
                    $state.SerialPort.Close()
                }
            } catch {
            } finally {
                $state.SerialPort.Dispose()
                $state.SerialPort = $null
            }
        }

        $script:RobotHandGui.ConnectButton.Text = "Connect"
        & $setStatusAction "Disconnected"
    }.GetNewClosure()

    $refreshPortsAction = {
        $currentPort = if ($state.SerialPort -and $state.SerialPort.IsOpen) { $state.SerialPort.PortName } else { $Port }
        $ports = @(Get-Ports)
        $portCombo.Items.Clear()
        $portMap.Clear()

        foreach ($entry in $ports) {
            $label = "{0} - {1}" -f $entry.Device, $entry.Description
            [void]$portCombo.Items.Add($label)
            $portMap[$label] = $entry.Device
        }

        if ($ports.Count -eq 0) {
            $portCombo.Text = ""
            & $setStatusAction "No serial ports found"
            return
        }

        $preferred = $null
        if ($currentPort) {
            foreach ($label in $portMap.Keys) {
                if ($portMap[$label] -eq $currentPort) {
                    $preferred = $label
                    break
                }
            }
        }

        if (-not $preferred) {
            $preferred = [string]$portCombo.Items[0]
        }

        $portCombo.SelectedItem = $preferred
        & $setStatusAction ("Found {0} serial port(s)" -f $ports.Count)
    }.GetNewClosure()

    $connectPortAction = {
        $portName = & $getSelectedPortAction
        if (-not $portName) {
            & $appendSystemLogAction "Select a serial port first."
            & $setStatusAction "No port selected"
            return
        }

        $parsedBaud = 0
        if (-not [int]::TryParse($baudCombo.Text, [ref]$parsedBaud)) {
            & $appendSystemLogAction ("Invalid baud rate: {0}" -f $baudCombo.Text)
            & $setStatusAction "Invalid baud rate"
            return
        }

        & $disconnectPortAction

        try {
            $newPort = [System.IO.Ports.SerialPort]::new($portName, $parsedBaud)
            $newPort.Encoding = [System.Text.Encoding]::UTF8
            $newPort.NewLine = $lineEndingMap[(& $getLineEndingAction)]
            $newPort.ReadTimeout = 25
            $newPort.WriteTimeout = 1000
            $newPort.Open()
            $state.SerialPort = $newPort

            if (-not $state.PollTimer) {
                $state.PollTimer = New-Object System.Windows.Forms.Timer
                $state.PollTimer.Interval = 50
                $state.PollTimer.Add_Tick({
                    if (-not $state.SerialPort -or -not $state.SerialPort.IsOpen) {
                        return
                    }

                    try {
                        $text = $state.SerialPort.ReadExisting()
                        if (-not [string]::IsNullOrEmpty($text)) {
                            & $appendRxLogAction "RX" $text
                        }
                    } catch {
                        & $appendSystemLogAction ("Serial read stopped: {0}" -f $_.Exception.Message)
                        & $disconnectPortAction
                    }
                }.GetNewClosure())
            }

            $state.PollTimer.Start()

            $script:RobotHandGui.ConnectButton.Text = "Disconnect"
            & $appendSystemLogAction ("Connected to {0} at {1} baud" -f $portName, $parsedBaud)
            & $setStatusAction ("Connected to {0}" -f $portName)
            $script:RobotHandGui.InputBox.Focus()
        } catch {
            & $appendSystemLogAction ("Failed to open {0}: {1}" -f $portName, $_.Exception.Message)
            & $setStatusAction ("Failed to open {0}" -f $portName)
        }
    }.GetNewClosure()

    $sendLineAction = {
        param([string]$Text, [bool]$KeepInput = $false)

        if (-not $PSBoundParameters.ContainsKey('Text')) {
            $Text = if ($script:RobotHandGui -and $script:RobotHandGui.InputBox) {
                $script:RobotHandGui.InputBox.Text
            } else {
                ""
            }
        }

        $text = if ($null -eq $Text) { "" } else { $Text.Trim() }
        if ([string]::IsNullOrEmpty($text)) {
            return
        }

        if (-not $state.SerialPort -or -not $state.SerialPort.IsOpen) {
            & $appendSystemLogAction "No serial port is open."
            & $setStatusAction "No serial port is open"
            return
        }

        try {
            $payload = $text + $lineEndingMap[(& $getLineEndingAction)]
            $state.SerialPort.Write($payload)
            & $appendTxLogAction "TX" $text
            if (-not $KeepInput) {
                $script:RobotHandGui.InputBox.Clear()
            }
        } catch {
            & $appendSystemLogAction ("Send failed: {0}" -f $_.Exception.Message)
            & $disconnectPortAction
        }
    }.GetNewClosure()

    $quickCommands = @(
        @{ Label = "Help"; Command = "h" }
        @{ Label = "Calibrate Direction"; Command = "cd" }
        @{ Label = "Calibrate Position"; Command = "cp" }
        @{ Label = "Encoders"; Command = "e" }
        @{ Label = "Reset All"; Command = "ra" }
        @{ Label = "Move Zero"; Command = "mz" }
        @{ Label = "Stop All"; Command = "sa" }
        @{ Label = "Finger Demo"; Command = "fd" }
        @{ Label = "Peace"; Command = "fp" }
        @{ Label = "Rock On"; Command = "fr" }
        @{ Label = "Middle"; Command = "fm" }
        @{ Label = "Stop M1"; Command = "s1" }
        @{ Label = "Stop M2"; Command = "s2" }
        @{ Label = "Stop M3"; Command = "s3" }
        @{ Label = "Stop M4"; Command = "s4" }
        @{ Label = "Stop M5"; Command = "s5" }
        @{ Label = "Stop M6"; Command = "s6" }
        @{ Label = "Stop M7"; Command = "s7" }
        @{ Label = "Stop M8"; Command = "s8" }
    )

    foreach ($item in $quickCommands) {
        $quickLayout.Controls.Add((New-CommandButton $item.Label $item.Command))
    }

    $jogGroup = Add-BuilderGroup "Jog / Relative Move" 186

    $jogMotorField = New-LabeledTextBox "Motor (1-8)" "1" 12 28
    $jogGroup.Controls.Add($jogMotorField.Label)
    $jogGroup.Controls.Add($jogMotorField.TextBox)

    $jogDirectionLabel = New-Object System.Windows.Forms.Label
    $jogDirectionLabel.Text = "Direction"
    $jogDirectionLabel.Location = New-Object System.Drawing.Point(12, 62)
    $jogDirectionLabel.AutoSize = $true
    $jogGroup.Controls.Add($jogDirectionLabel)

    $jogDirectionCombo = New-Object System.Windows.Forms.ComboBox
    $jogDirectionCombo.Location = New-Object System.Drawing.Point(88, 58)
    $jogDirectionCombo.Size = New-Object System.Drawing.Size(96, 24)
    $jogDirectionCombo.DropDownStyle = "DropDownList"
    [void]$jogDirectionCombo.Items.AddRange(@("f", "b"))
    $jogDirectionCombo.SelectedIndex = 0
    $jogGroup.Controls.Add($jogDirectionCombo)

    $jogCountsField = New-LabeledTextBox "Counts" "" 12 92
    $jogGroup.Controls.Add($jogCountsField.Label)
    $jogGroup.Controls.Add($jogCountsField.TextBox)

    $jogToleranceField = New-LabeledTextBox "Tolerance" "5" 12 126
    $jogGroup.Controls.Add($jogToleranceField.Label)
    $jogGroup.Controls.Add($jogToleranceField.TextBox)

    $jogSendButton = New-Object System.Windows.Forms.Button
    $jogSendButton.Text = "Send Jog"
    $jogSendButton.Location = New-Object System.Drawing.Point(208, 56)
    $jogSendButton.Size = New-Object System.Drawing.Size(92, 30)
    $jogSendButton.Add_Click({
        $motorId = $jogMotorField.TextBox.Text.Trim()
        $direction = [string]$jogDirectionCombo.SelectedItem
        $counts = $jogCountsField.TextBox.Text.Trim()
        $tolerance = $jogToleranceField.TextBox.Text.Trim()

        if ([string]::IsNullOrWhiteSpace($motorId) -or [string]::IsNullOrWhiteSpace($direction)) {
            & $appendSystemLogAction "Jog command needs a motor and direction."
            return
        }

        $commandText = if ([string]::IsNullOrWhiteSpace($counts)) {
            "j{0}{1}" -f $motorId, $direction
        } else {
            if ([string]::IsNullOrWhiteSpace($tolerance)) {
                $tolerance = "5"
            }
            "j{0}{1},{2},{3}" -f $motorId, $direction, $counts, $tolerance
        }

        & $sendLineAction $commandText
    }.GetNewClosure())
    $jogGroup.Controls.Add($jogSendButton)

    $positionGroup = Add-BuilderGroup "Move To Position" 150

    $positionMotorField = New-LabeledTextBox "Motor (1-8)" "1" 12 28
    $positionGroup.Controls.Add($positionMotorField.Label)
    $positionGroup.Controls.Add($positionMotorField.TextBox)

    $positionValueField = New-LabeledTextBox "Position" "0" 12 62
    $positionGroup.Controls.Add($positionValueField.Label)
    $positionGroup.Controls.Add($positionValueField.TextBox)

    $positionToleranceField = New-LabeledTextBox "Tolerance" "5" 12 96
    $positionGroup.Controls.Add($positionToleranceField.Label)
    $positionGroup.Controls.Add($positionToleranceField.TextBox)

    $positionSendButton = New-Object System.Windows.Forms.Button
    $positionSendButton.Text = "Send Position"
    $positionSendButton.Location = New-Object System.Drawing.Point(208, 58)
    $positionSendButton.Size = New-Object System.Drawing.Size(92, 30)
    $positionSendButton.Add_Click({
        $commandText = "p{0},{1},{2}" -f `
            $positionMotorField.TextBox.Text.Trim(), `
            $positionValueField.TextBox.Text.Trim(), `
            $positionToleranceField.TextBox.Text.Trim()
        & $sendLineAction $commandText
    }.GetNewClosure())
    $positionGroup.Controls.Add($positionSendButton)

    $fingerGroup = Add-BuilderGroup "Finger Move" 184

    $fingerIdField = New-LabeledTextBox "Finger (1-4)" "1" 12 28
    $fingerGroup.Controls.Add($fingerIdField.Label)
    $fingerGroup.Controls.Add($fingerIdField.TextBox)

    $fingerProxField = New-LabeledTextBox "Proximal" "0" 12 62
    $fingerGroup.Controls.Add($fingerProxField.Label)
    $fingerGroup.Controls.Add($fingerProxField.TextBox)

    $fingerDistField = New-LabeledTextBox "Distal" "0" 12 96
    $fingerGroup.Controls.Add($fingerDistField.Label)
    $fingerGroup.Controls.Add($fingerDistField.TextBox)

    $fingerToleranceField = New-LabeledTextBox "Tolerance" "5" 12 130
    $fingerGroup.Controls.Add($fingerToleranceField.Label)
    $fingerGroup.Controls.Add($fingerToleranceField.TextBox)

    $fingerSendButton = New-Object System.Windows.Forms.Button
    $fingerSendButton.Text = "Send Finger"
    $fingerSendButton.Location = New-Object System.Drawing.Point(208, 74)
    $fingerSendButton.Size = New-Object System.Drawing.Size(92, 30)
    $fingerSendButton.Add_Click({
        $fingerTolerance = $fingerToleranceField.TextBox.Text.Trim()
        if ([string]::IsNullOrWhiteSpace($fingerTolerance)) {
            $fingerTolerance = "5"
        }
        $commandText = "f{0},{1},{2},{3}" -f `
            $fingerIdField.TextBox.Text.Trim(), `
            $fingerProxField.TextBox.Text.Trim(), `
            $fingerDistField.TextBox.Text.Trim(), `
            $fingerTolerance
        & $sendLineAction $commandText
    }.GetNewClosure())
    $fingerGroup.Controls.Add($fingerSendButton)

    $stopGroup = Add-BuilderGroup "Stop Single Motor" 96

    $stopMotorField = New-LabeledTextBox "Motor (1-8)" "1" 12 32
    $stopGroup.Controls.Add($stopMotorField.Label)
    $stopGroup.Controls.Add($stopMotorField.TextBox)

    $stopSendButton = New-Object System.Windows.Forms.Button
    $stopSendButton.Text = "Send Stop"
    $stopSendButton.Location = New-Object System.Drawing.Point(208, 30)
    $stopSendButton.Size = New-Object System.Drawing.Size(92, 30)
    $stopSendButton.Add_Click({
        $commandText = "s{0}" -f $stopMotorField.TextBox.Text.Trim()
        & $sendLineAction $commandText
    }.GetNewClosure())
    $stopGroup.Controls.Add($stopSendButton)

    $refreshButton.Add_Click({ & $refreshPortsAction }.GetNewClosure())
    $connectButton.Add_Click({
        if ($state.SerialPort -and $state.SerialPort.IsOpen) {
            & $disconnectPortAction
        } else {
            & $connectPortAction
        }
    }.GetNewClosure())
    $sendButton.Add_Click({ & $sendLineAction }.GetNewClosure())
    $sendSelectedButton.Add_Click({
        if ($script:RobotHandGui.InputBox.SelectionLength -gt 0) {
            & $sendLineAction $script:RobotHandGui.InputBox.SelectedText $true
        } else {
            & $sendLineAction
        }
    }.GetNewClosure())
    $clearRxButton.Add_Click({ $rxLogBox.Clear() }.GetNewClosure())
    $clearTxButton.Add_Click({ $txLogBox.Clear() }.GetNewClosure())
    $inputBox.Add_KeyDown({
        if ($_.KeyCode -eq [System.Windows.Forms.Keys]::Enter) {
            $_.SuppressKeyPress = $true
            & $sendLineAction
        }
    }.GetNewClosure())
    $endingCombo.Add_SelectedIndexChanged({
        if ($state.SerialPort -and $state.SerialPort.IsOpen) {
            $state.SerialPort.NewLine = $lineEndingMap[(& $getLineEndingAction)]
        }
    }.GetNewClosure())
    $form.Add_Shown({
        $logSplit.Panel1MinSize = 180
        $logSplit.Panel2MinSize = 180
        $availableHeight = $logSplit.ClientSize.Height
        if ($availableHeight -gt ($logSplit.Panel1MinSize + $logSplit.Panel2MinSize)) {
            $preferredTop = [Math]::Min(310, $availableHeight - $logSplit.Panel2MinSize)
            $preferredTop = [Math]::Max($preferredTop, $logSplit.Panel1MinSize)
            $logSplit.SplitterDistance = $preferredTop
        }
        & $refreshPortsAction
        if ($Port) {
            & $connectPortAction
        }
        $script:RobotHandGui.InputBox.Focus()
    }.GetNewClosure())
    $form.Add_FormClosing({
        & $disconnectPortAction
    }.GetNewClosure())

    [void]$form.ShowDialog()
}

if ($Help) {
    Show-Usage
    return
}

if ($Cli) {
    Start-CliTerminal
    return
}

Start-GuiTerminal
