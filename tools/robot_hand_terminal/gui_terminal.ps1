function Start-RobotHandGui {
    param(
        [string]$Port,
        [int]$BaudRate,
        [string]$LineEnding,
        [hashtable]$LineEndingMap,
        [scriptblock]$GetPortsBlock
    )

    [System.Windows.Forms.Application]::EnableVisualStyles()

    $script:RobotHandGuiState = [ordered]@{
        SerialPort = $null
        PollTimer = $null
        PortMap = @{}
        LineEndingMap = $LineEndingMap
        LastCommand = ""
        RxBuffer = ""
        QuickGroups = @()
        QuickGroupLayouts = @{}
        BuilderGroups = @()
        SliderControls = @{}
        SuppressSliderSync = $false
    }

    $form = New-Object System.Windows.Forms.Form
    $form.Text = "Robot Hand Serial Terminal"
    $form.StartPosition = "CenterScreen"
    $form.Size = New-Object System.Drawing.Size(1320, 820)
    $form.MinimumSize = New-Object System.Drawing.Size(1120, 700)
    $script:RobotHandGuiState.Form = $form

    $rootLayout = New-Object System.Windows.Forms.TableLayoutPanel
    $rootLayout.Dock = "Fill"
    $rootLayout.ColumnCount = 1
    $rootLayout.RowCount = 3
    $rootLayout.Padding = New-Object System.Windows.Forms.Padding(12)
    [void]$rootLayout.ColumnStyles.Add((New-Object System.Windows.Forms.ColumnStyle([System.Windows.Forms.SizeType]::Percent, 100)))
    [void]$rootLayout.RowStyles.Add((New-Object System.Windows.Forms.RowStyle([System.Windows.Forms.SizeType]::Absolute, 108)))
    [void]$rootLayout.RowStyles.Add((New-Object System.Windows.Forms.RowStyle([System.Windows.Forms.SizeType]::Percent, 100)))
    [void]$rootLayout.RowStyles.Add((New-Object System.Windows.Forms.RowStyle([System.Windows.Forms.SizeType]::Absolute, 96)))
    $form.Controls.Add($rootLayout)

    $topPanel = New-Object System.Windows.Forms.Panel
    $topPanel.Dock = "Fill"
    $topPanel.Margin = New-Object System.Windows.Forms.Padding(0, 0, 0, 12)
    $rootLayout.Controls.Add($topPanel, 0, 0)

    $serialGroup = New-Object System.Windows.Forms.GroupBox
    $serialGroup.Dock = "Fill"
    $serialGroup.Text = "Serial Connection"
    $serialGroup.Padding = New-Object System.Windows.Forms.Padding(10, 24, 10, 10)
    $topPanel.Controls.Add($serialGroup)

    $statusLabel = New-Object System.Windows.Forms.Label
    $statusLabel.Dock = "Bottom"
    $statusLabel.Height = 24
    $statusLabel.Padding = New-Object System.Windows.Forms.Padding(8, 4, 8, 4)
    $statusLabel.BorderStyle = "Fixed3D"
    $statusLabel.Text = "Disconnected"
    $statusLabel.Margin = New-Object System.Windows.Forms.Padding(0)
    $serialGroup.Controls.Add($statusLabel)
    $script:RobotHandGuiState.StatusLabel = $statusLabel

    $bottomPanel = New-Object System.Windows.Forms.Panel
    $bottomPanel.Dock = "Fill"
    $bottomPanel.Margin = New-Object System.Windows.Forms.Padding(0, 12, 0, 0)
    $rootLayout.Controls.Add($bottomPanel, 0, 2)

    $commandLineGroup = New-Object System.Windows.Forms.GroupBox
    $commandLineGroup.Dock = "Fill"
    $commandLineGroup.Text = "Command Line"
    $commandLineGroup.Padding = New-Object System.Windows.Forms.Padding(10, 24, 10, 10)
    $bottomPanel.Controls.Add($commandLineGroup)

    $contentPanel = New-Object System.Windows.Forms.Panel
    $contentPanel.Dock = "Fill"
    $contentPanel.Margin = New-Object System.Windows.Forms.Padding(0)
    $rootLayout.Controls.Add($contentPanel, 0, 1)

    $contentSplit = New-Object System.Windows.Forms.SplitContainer
    $contentSplit.Dock = "Fill"
    $contentSplit.Orientation = "Vertical"
    $contentPanel.Controls.Add($contentSplit)
    $script:RobotHandGuiState.ContentSplit = $contentSplit

    $mainPanel = New-Object System.Windows.Forms.Panel
    $mainPanel.Dock = "Fill"
    $mainPanel.Padding = New-Object System.Windows.Forms.Padding(0, 0, 6, 0)
    $contentSplit.Panel1.Controls.Add($mainPanel)

    $commandPanel = New-Object System.Windows.Forms.Panel
    $commandPanel.Dock = "Fill"
    $commandPanel.Padding = New-Object System.Windows.Forms.Padding(6, 0, 0, 0)
    $contentSplit.Panel2.Controls.Add($commandPanel)

    $logSplit = New-Object System.Windows.Forms.SplitContainer
    $logSplit.Dock = "Fill"
    $logSplit.Orientation = "Horizontal"
    $mainPanel.Controls.Add($logSplit)
    $script:RobotHandGuiState.LogSplit = $logSplit

    $logSplit.Panel1.Padding = New-Object System.Windows.Forms.Padding(0)
    $logSplit.Panel2.Padding = New-Object System.Windows.Forms.Padding(0, 8, 0, 0)

    $rxGroup = New-Object System.Windows.Forms.GroupBox
    $rxGroup.Dock = "Fill"
    $rxGroup.Text = "Incoming Data (RX)"
    $rxGroup.Padding = New-Object System.Windows.Forms.Padding(10, 24, 10, 10)
    $rxGroup.Margin = New-Object System.Windows.Forms.Padding(0)
    $logSplit.Panel1.Controls.Add($rxGroup)

    $rxLogBox = New-Object System.Windows.Forms.RichTextBox
    $rxLogBox.Dock = "Fill"
    $rxLogBox.ReadOnly = $true
    $rxLogBox.HideSelection = $false
    $rxLogBox.BackColor = [System.Drawing.Color]::White
    $rxLogBox.Font = New-Object System.Drawing.Font("Consolas", 10)
    $rxGroup.Controls.Add($rxLogBox)
    $script:RobotHandGuiState.RxLogBox = $rxLogBox

    $txGroup = New-Object System.Windows.Forms.GroupBox
    $txGroup.Dock = "Fill"
    $txGroup.Text = "Outgoing Data / System (TX)"
    $txGroup.Padding = New-Object System.Windows.Forms.Padding(10, 24, 10, 10)
    $txGroup.Margin = New-Object System.Windows.Forms.Padding(0)
    $logSplit.Panel2.Controls.Add($txGroup)

    $txLogBox = New-Object System.Windows.Forms.RichTextBox
    $txLogBox.Dock = "Fill"
    $txLogBox.ReadOnly = $true
    $txLogBox.HideSelection = $false
    $txLogBox.BackColor = [System.Drawing.Color]::White
    $txLogBox.Font = New-Object System.Drawing.Font("Consolas", 10)
    $txGroup.Controls.Add($txLogBox)
    $script:RobotHandGuiState.TxLogBox = $txLogBox

    $commandTabs = New-Object System.Windows.Forms.TabControl
    $commandTabs.Dock = "Fill"
    $commandPanel.Controls.Add($commandTabs)

    $quickTab = New-Object System.Windows.Forms.TabPage
    $quickTab.Text = "Quick Commands"
    [void]$commandTabs.TabPages.Add($quickTab)

    $buildersTab = New-Object System.Windows.Forms.TabPage
    $buildersTab.Text = "Command Builders"
    [void]$commandTabs.TabPages.Add($buildersTab)

    $slidersTab = New-Object System.Windows.Forms.TabPage
    $slidersTab.Text = "Sliders"
    [void]$commandTabs.TabPages.Add($slidersTab)

    $quickLayout = New-Object System.Windows.Forms.FlowLayoutPanel
    $quickLayout.Dock = "Fill"
    $quickLayout.FlowDirection = "TopDown"
    $quickLayout.WrapContents = $false
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

    $slidersRootPanel = New-Object System.Windows.Forms.Panel
    $slidersRootPanel.Dock = "Fill"
    $slidersRootPanel.Padding = New-Object System.Windows.Forms.Padding(8)
    $slidersTab.Controls.Add($slidersRootPanel)

    $positionGroup = New-Object System.Windows.Forms.GroupBox
    $positionGroup.Dock = "Fill"
    $positionGroup.Text = "Position"
    $positionGroup.Padding = New-Object System.Windows.Forms.Padding(10, 24, 10, 10)
    $slidersRootPanel.Controls.Add($positionGroup)

    $speedGroup = New-Object System.Windows.Forms.GroupBox
    $speedGroup.Dock = "Top"
    $speedGroup.Height = 132
    $speedGroup.Text = "Speed"
    $speedGroup.Padding = New-Object System.Windows.Forms.Padding(10, 34, 10, 12)
    $speedGroup.Margin = New-Object System.Windows.Forms.Padding(0, 0, 0, 10)
    $slidersRootPanel.Controls.Add($speedGroup)

    $sliderTopPanel = New-Object System.Windows.Forms.Panel
    $sliderTopPanel.Dock = "Top"
    $sliderTopPanel.Height = 40
    $positionGroup.Controls.Add($sliderTopPanel)

    $sliderPanel = New-Object System.Windows.Forms.Panel
    $sliderPanel.Dock = "Fill"
    $sliderPanel.AutoScroll = $true
    $positionGroup.Controls.Add($sliderPanel)

    $sliderUpdateButton = New-Object System.Windows.Forms.Button
    $sliderUpdateButton.Text = "Update"
    $sliderUpdateButton.Size = New-Object System.Drawing.Size(88, 28)
    $sliderUpdateButton.Location = New-Object System.Drawing.Point(0, 0)
    $sliderTopPanel.Controls.Add($sliderUpdateButton)

    $speedLabel = New-Object System.Windows.Forms.Label
    $speedLabel.Text = "Movement Speed"
    $speedLabel.Location = New-Object System.Drawing.Point(10, 18)
    $speedLabel.AutoSize = $true
    $speedGroup.Controls.Add($speedLabel)

    $speedValueLabel = New-Object System.Windows.Forms.Label
    $speedValueLabel.Text = "170"
    $speedValueLabel.Location = New-Object System.Drawing.Point(122, 18)
    $speedValueLabel.Size = New-Object System.Drawing.Size(56, 20)
    $speedGroup.Controls.Add($speedValueLabel)

    $speedMinLabel = New-Object System.Windows.Forms.Label
    $speedMinLabel.Text = "60"
    $speedMinLabel.Location = New-Object System.Drawing.Point(10, 86)
    $speedMinLabel.AutoSize = $true
    $speedGroup.Controls.Add($speedMinLabel)

    $speedMaxLabel = New-Object System.Windows.Forms.Label
    $speedMaxLabel.Text = "255"
    $speedMaxLabel.AutoSize = $true
    $speedGroup.Controls.Add($speedMaxLabel)

    $speedTrackBar = New-Object System.Windows.Forms.TrackBar
    $speedTrackBar.Minimum = 60
    $speedTrackBar.Maximum = 255
    $speedTrackBar.Value = 170
    $speedTrackBar.TickFrequency = 10
    $speedTrackBar.SmallChange = 50
    $speedTrackBar.LargeChange = 50
    $speedTrackBar.AutoSize = $false
    $speedTrackBar.Height = 30
    $speedTrackBar.Width = 250
    $speedTrackBar.Location = New-Object System.Drawing.Point(44, 78)
    $speedGroup.Controls.Add($speedTrackBar)

    $speedGroup.Add_SizeChanged({
        $innerWidth = $speedGroup.ClientSize.Width
        $speedTrackBar.Width = [Math]::Max(120, $innerWidth - 100)
        $speedMaxLabel.Left = [Math]::Max($speedTrackBar.Right + 8, $innerWidth - $speedMaxLabel.PreferredWidth - 6)
        $speedMaxLabel.Top = $speedMinLabel.Top
    })

    $speedTrackBar.Add_MouseWheel({
        param($sender, $eventArgs)
        $step = 50
        $relative = $sender.Value - $sender.Minimum
        $baseValue = $sender.Minimum + ([int][Math]::Floor($relative / $step) * $step)
        $targetValue = if ($eventArgs.Delta -gt 0) { $baseValue + $step } else { $baseValue }
        $sender.Value = [Math]::Max($sender.Minimum, [Math]::Min($sender.Maximum, $targetValue))
    })

    $sliderLayout = New-Object System.Windows.Forms.TableLayoutPanel
    $sliderLayout.Dock = "Top"
    $sliderLayout.AutoSize = $true
    $sliderLayout.AutoSizeMode = [System.Windows.Forms.AutoSizeMode]::GrowAndShrink
    $sliderLayout.ColumnCount = 1
    $sliderLayout.RowCount = 9
    $sliderLayout.Padding = New-Object System.Windows.Forms.Padding(8)
    [void]$sliderLayout.ColumnStyles.Add((New-Object System.Windows.Forms.ColumnStyle([System.Windows.Forms.SizeType]::Percent, 100)))
    $sliderPanel.Controls.Add($sliderLayout)

    $sliderHintLabel = New-Object System.Windows.Forms.Label
    $sliderHintLabel.Text = "Move a slider to send that motor to the selected position. Press Update to refresh slider positions from the hand."
    $sliderHintLabel.Dock = "Fill"
    $sliderHintLabel.AutoSize = $true
    $sliderHintLabel.Margin = New-Object System.Windows.Forms.Padding(3, 3, 3, 10)
    $sliderLayout.Controls.Add($sliderHintLabel, 0, 0)

    $portLabel = New-Object System.Windows.Forms.Label
    $portLabel.Text = "Port"
    $portLabel.Location = New-Object System.Drawing.Point(12, 18)
    $portLabel.AutoSize = $true
    $serialGroup.Controls.Add($portLabel)

    $portCombo = New-Object System.Windows.Forms.ComboBox
    $portCombo.Location = New-Object System.Drawing.Point(48, 14)
    $portCombo.Size = New-Object System.Drawing.Size(320, 24)
    $portCombo.DropDownStyle = "DropDownList"
    $serialGroup.Controls.Add($portCombo)
    $script:RobotHandGuiState.PortCombo = $portCombo

    $refreshButton = New-Object System.Windows.Forms.Button
    $refreshButton.Text = "Refresh"
    $refreshButton.Location = New-Object System.Drawing.Point(380, 12)
    $refreshButton.Size = New-Object System.Drawing.Size(78, 28)
    $serialGroup.Controls.Add($refreshButton)

    $connectButton = New-Object System.Windows.Forms.Button
    $connectButton.Text = "Connect"
    $connectButton.Location = New-Object System.Drawing.Point(466, 12)
    $connectButton.Size = New-Object System.Drawing.Size(88, 28)
    $serialGroup.Controls.Add($connectButton)
    $script:RobotHandGuiState.ConnectButton = $connectButton

    $baudLabel = New-Object System.Windows.Forms.Label
    $baudLabel.Text = "Baud"
    $baudLabel.Location = New-Object System.Drawing.Point(568, 18)
    $baudLabel.AutoSize = $true
    $serialGroup.Controls.Add($baudLabel)

    $baudCombo = New-Object System.Windows.Forms.ComboBox
    $baudCombo.Location = New-Object System.Drawing.Point(608, 14)
    $baudCombo.Size = New-Object System.Drawing.Size(96, 24)
    $baudCombo.DropDownStyle = "DropDown"
    [void]$baudCombo.Items.AddRange(@("9600", "57600", "115200", "230400"))
    $baudCombo.Text = [string]$BaudRate
    $serialGroup.Controls.Add($baudCombo)
    $script:RobotHandGuiState.BaudCombo = $baudCombo

    $endingLabel = New-Object System.Windows.Forms.Label
    $endingLabel.Text = "Line Ending"
    $endingLabel.Location = New-Object System.Drawing.Point(720, 18)
    $endingLabel.AutoSize = $true
    $serialGroup.Controls.Add($endingLabel)

    $endingCombo = New-Object System.Windows.Forms.ComboBox
    $endingCombo.Location = New-Object System.Drawing.Point(798, 14)
    $endingCombo.Size = New-Object System.Drawing.Size(96, 24)
    $endingCombo.DropDownStyle = "DropDownList"
    [void]$endingCombo.Items.AddRange(@("none", "lf", "cr", "crlf"))
    $endingCombo.SelectedItem = $LineEnding
    if ($null -eq $endingCombo.SelectedItem) { $endingCombo.SelectedItem = "lf" }
    $serialGroup.Controls.Add($endingCombo)
    $script:RobotHandGuiState.EndingCombo = $endingCombo

    $inputBox = New-Object System.Windows.Forms.TextBox
    $inputBox.Location = New-Object System.Drawing.Point(12, 18)
    $inputBox.Size = New-Object System.Drawing.Size(860, 24)
    $commandLineGroup.Controls.Add($inputBox)
    $script:RobotHandGuiState.InputBox = $inputBox

    $sendButton = New-Object System.Windows.Forms.Button
    $sendButton.Text = "Send"
    $sendButton.Location = New-Object System.Drawing.Point(884, 16)
    $sendButton.Size = New-Object System.Drawing.Size(78, 28)
    $commandLineGroup.Controls.Add($sendButton)

    $clearRxButton = New-Object System.Windows.Forms.Button
    $clearRxButton.Text = "Clear RX"
    $clearRxButton.Location = New-Object System.Drawing.Point(972, 16)
    $clearRxButton.Size = New-Object System.Drawing.Size(82, 28)
    $commandLineGroup.Controls.Add($clearRxButton)

    $clearTxButton = New-Object System.Windows.Forms.Button
    $clearTxButton.Text = "Clear TX"
    $clearTxButton.Location = New-Object System.Drawing.Point(1064, 16)
    $clearTxButton.Size = New-Object System.Drawing.Size(82, 28)
    $commandLineGroup.Controls.Add($clearTxButton)

    $hintLabel = New-Object System.Windows.Forms.Label
    $hintLabel.Location = New-Object System.Drawing.Point(12, 54)
    $hintLabel.Size = New-Object System.Drawing.Size(1240, 24)
    $hintLabel.Text = "Type any raw command below, or use the quick buttons and builders on the right."
    $commandLineGroup.Controls.Add($hintLabel)

    $script:SetRobotHandGuiStatus = {
        param([string]$Message)
        $script:RobotHandGuiState.StatusLabel.Text = $Message
    }

    $script:AppendRobotHandGuiLog = {
        param([string]$Tag, [string]$Message)
        $box = if ($Tag -eq "RX") { $script:RobotHandGuiState.RxLogBox } else { $script:RobotHandGuiState.TxLogBox }
        $timestamp = Get-Date -Format "HH:mm:ss"
        $box.AppendText(("[{0}] [{1}] {2}" -f $timestamp, $Tag, $Message))
        if (-not $Message.EndsWith("`n")) {
            $box.AppendText("`r`n")
        }
        $box.SelectionStart = $box.TextLength
        $box.SelectionLength = 0
        $box.ScrollToCaret()
    }

    $script:GetRobotHandGuiLineEnding = {
        $selected = [string]$script:RobotHandGuiState.EndingCombo.SelectedItem
        if ([string]::IsNullOrWhiteSpace($selected)) {
            $selected = [string]$script:RobotHandGuiState.EndingCombo.Text
        }
        if (-not $script:RobotHandGuiState.LineEndingMap.ContainsKey($selected)) {
            $selected = "lf"
        }
        return $selected
    }

    $script:DisconnectRobotHandGuiPort = {
        if ($script:RobotHandGuiState.PollTimer) {
            $script:RobotHandGuiState.PollTimer.Stop()
        }
        if ($script:RobotHandGuiState.SerialPort) {
            try {
                if ($script:RobotHandGuiState.SerialPort.IsOpen) {
                    $script:RobotHandGuiState.SerialPort.Close()
                }
            } catch {
            } finally {
                $script:RobotHandGuiState.SerialPort.Dispose()
                $script:RobotHandGuiState.SerialPort = $null
            }
        }
        $script:RobotHandGuiState.ConnectButton.Text = "Connect"
        $script:RobotHandGuiState.RxBuffer = ""
        & $script:SetRobotHandGuiStatus "Disconnected"
    }

    $script:RefreshRobotHandGuiPorts = {
        $currentPort = if ($script:RobotHandGuiState.SerialPort -and $script:RobotHandGuiState.SerialPort.IsOpen) { $script:RobotHandGuiState.SerialPort.PortName } else { $Port }
        $ports = @(& $GetPortsBlock)
        $script:RobotHandGuiState.PortCombo.Items.Clear()
        $script:RobotHandGuiState.PortMap.Clear()
        foreach ($entry in $ports) {
            $label = "{0} - {1}" -f $entry.Device, $entry.Description
            [void]$script:RobotHandGuiState.PortCombo.Items.Add($label)
            $script:RobotHandGuiState.PortMap[$label] = $entry.Device
        }
        if ($ports.Count -eq 0) {
            $script:RobotHandGuiState.PortCombo.Text = ""
            & $script:SetRobotHandGuiStatus "No serial ports found"
            return
        }
        $preferred = $null
        if ($currentPort) {
            foreach ($label in $script:RobotHandGuiState.PortMap.Keys) {
                if ($script:RobotHandGuiState.PortMap[$label] -eq $currentPort) {
                    $preferred = $label
                    break
                }
            }
        }
        if (-not $preferred) {
            $preferred = [string]$script:RobotHandGuiState.PortCombo.Items[0]
        }
        $script:RobotHandGuiState.PortCombo.SelectedItem = $preferred
        & $script:SetRobotHandGuiStatus ("Found {0} serial port(s)" -f $ports.Count)
    }

    $script:GetRobotHandSelectedPort = {
        $selection = [string]$script:RobotHandGuiState.PortCombo.SelectedItem
        if ([string]::IsNullOrWhiteSpace($selection)) { $selection = [string]$script:RobotHandGuiState.PortCombo.Text }
        if ([string]::IsNullOrWhiteSpace($selection)) { return $null }
        if ($script:RobotHandGuiState.PortMap.ContainsKey($selection)) { return $script:RobotHandGuiState.PortMap[$selection] }
        return ($selection -split ' - ', 2)[0]
    }

    $script:RequestRobotHandSliderSnapshot = {
        if (-not $script:RobotHandGuiState.SerialPort -or -not $script:RobotHandGuiState.SerialPort.IsOpen) {
            & $script:AppendRobotHandGuiLog "SYS" "No serial port is open."
            & $script:SetRobotHandGuiStatus "No serial port is open"
            return
        }
        try {
            $ending = & $script:GetRobotHandGuiLineEnding
            $lineEndingMap = $script:RobotHandGuiState.LineEndingMap
            if ($null -eq $lineEndingMap) {
                $lineEndingMap = @{
                    "none" = ""
                    "lf" = "`n"
                    "cr" = "`r"
                    "crlf" = "`r`n"
                }
            }
            $suffix = if ($lineEndingMap.ContainsKey($ending)) { $lineEndingMap[$ending] } else { $lineEndingMap["lf"] }
            $script:RobotHandGuiState.SerialPort.Write("es" + $suffix)
            & $script:AppendRobotHandGuiLog "TX" "es"
        } catch {
            & $script:AppendRobotHandGuiLog "SYS" ("Update failed: {0}" -f $_.Exception.Message)
            & $script:DisconnectRobotHandGuiPort
        }
    }

    $script:SendRobotHandGuiSpeedCommand = {
        param([int]$Speed)
        if (-not $script:RobotHandGuiState.SerialPort -or -not $script:RobotHandGuiState.SerialPort.IsOpen) {
            return
        }
        try {
            $ending = & $script:GetRobotHandGuiLineEnding
            $lineEndingMap = $script:RobotHandGuiState.LineEndingMap
            if ($null -eq $lineEndingMap) {
                $lineEndingMap = @{
                    "none" = ""
                    "lf" = "`n"
                    "cr" = "`r"
                    "crlf" = "`r`n"
                }
            }
            $suffix = if ($lineEndingMap.ContainsKey($ending)) { $lineEndingMap[$ending] } else { $lineEndingMap["lf"] }
            $commandText = "ms,{0}" -f $Speed
            $script:RobotHandGuiState.SerialPort.Write($commandText + $suffix)
            & $script:AppendRobotHandGuiLog "TX" $commandText
        } catch {
            & $script:AppendRobotHandGuiLog "SYS" ("Speed update failed: {0}" -f $_.Exception.Message)
        }
    }

    $script:UpdateMotorSlidersFromSnapshot = {
        param([string]$SnapshotLine)
        if (-not $SnapshotLine.StartsWith("@ENC ")) {
            return $false
        }

        $script:RobotHandGuiState.SuppressSliderSync = $true
        try {
            foreach ($entry in ($SnapshotLine.Substring(5) -split ';')) {
                if ([string]::IsNullOrWhiteSpace($entry)) {
                    continue
                }
                if ($entry -notmatch '^M(?<id>\d+)=(?<position>-?\d+),(?<min>-?\d+),(?<max>-?\d+)$') {
                    continue
                }
                $motorId = $matches['id']
                $position = [int]$matches['position']
                $minimum = [int]$matches['min']
                $maximum = [int]$matches['max']
                if (-not $script:RobotHandGuiState.SliderControls.ContainsKey($motorId)) {
                    continue
                }

                $slider = $script:RobotHandGuiState.SliderControls[$motorId]
                $slider.TrackBar.Minimum = $minimum
                $slider.TrackBar.Maximum = $maximum
                $slider.MinLabel.Text = [string]$minimum
                $slider.MaxLabel.Text = [string]$maximum
                $slider.TrackBar.AccessibleDescription = "initializing"
                $slider.TrackBar.Value = [Math]::Max($minimum, [Math]::Min($maximum, $position))
                $slider.TrackBar.AccessibleDescription = $null
                $slider.ValueLabel.Text = [string]$slider.TrackBar.Value
            }
        } finally {
            $script:RobotHandGuiState.SuppressSliderSync = $false
        }

        return $true
    }

    $script:HandleRobotHandIncomingData = {
        param([string]$IncomingText)
        if ([string]::IsNullOrEmpty($IncomingText)) {
            return
        }

        $script:RobotHandGuiState.RxBuffer += $IncomingText
        $normalized = $script:RobotHandGuiState.RxBuffer -replace "`r`n", "`n" -replace "`r", "`n"
        $lines = $normalized -split "`n", -1
        $script:RobotHandGuiState.RxBuffer = [string]$lines[-1]

        for ($lineIndex = 0; $lineIndex -lt ($lines.Count - 1); $lineIndex++) {
            $line = $lines[$lineIndex]
            if ([string]::IsNullOrWhiteSpace($line)) {
                continue
            }
            if (-not (& $script:UpdateMotorSlidersFromSnapshot $line)) {
                & $script:AppendRobotHandGuiLog "RX" $line
            }
        }
    }

    $script:SendRobotHandGuiCommand = {
        param([string]$CommandText, [switch]$KeepInput, [switch]$Silent)
        if (-not $PSBoundParameters.ContainsKey('CommandText')) {
            $CommandText = [string]$script:RobotHandGuiState.InputBox.Text
        }
        $text = if ($null -eq $CommandText) { "" } else { $CommandText.Trim() }
        if ([string]::IsNullOrWhiteSpace($text)) { return }
        if (-not $script:RobotHandGuiState.SerialPort -or -not $script:RobotHandGuiState.SerialPort.IsOpen) {
            & $script:AppendRobotHandGuiLog "SYS" "No serial port is open."
            & $script:SetRobotHandGuiStatus "No serial port is open"
            return
        }
        try {
            $ending = & $script:GetRobotHandGuiLineEnding
            $lineEndingMap = $script:RobotHandGuiState.LineEndingMap
            if ($null -eq $lineEndingMap) {
                $lineEndingMap = @{
                    "none" = ""
                    "lf" = "`n"
                    "cr" = "`r"
                    "crlf" = "`r`n"
                }
            }
            $suffix = if ($lineEndingMap.ContainsKey($ending)) { $lineEndingMap[$ending] } else { $lineEndingMap["lf"] }
            $script:RobotHandGuiState.SerialPort.Write($text + $suffix)
            if (-not $Silent) {
                $script:RobotHandGuiState.LastCommand = $text
                & $script:AppendRobotHandGuiLog "TX" $text
                if (-not $KeepInput) { $script:RobotHandGuiState.InputBox.Clear() }
            }
        } catch {
            & $script:AppendRobotHandGuiLog "SYS" ("Send failed: {0}" -f $_.Exception.Message)
            & $script:DisconnectRobotHandGuiPort
        }
    }

    $script:ConnectRobotHandGuiPort = {
        $portName = & $script:GetRobotHandSelectedPort
        if (-not $portName) {
            & $script:AppendRobotHandGuiLog "SYS" "Select a serial port first."
            & $script:SetRobotHandGuiStatus "No port selected"
            return
        }
        $parsedBaud = 0
        if (-not [int]::TryParse([string]$script:RobotHandGuiState.BaudCombo.Text, [ref]$parsedBaud)) {
            & $script:AppendRobotHandGuiLog "SYS" ("Invalid baud rate: {0}" -f $script:RobotHandGuiState.BaudCombo.Text)
            & $script:SetRobotHandGuiStatus "Invalid baud rate"
            return
        }
        & $script:DisconnectRobotHandGuiPort
        try {
            $portHandle = [System.IO.Ports.SerialPort]::new($portName, $parsedBaud)
            $portHandle.Encoding = [System.Text.Encoding]::UTF8
            $portHandle.NewLine = $script:RobotHandGuiState.LineEndingMap[(& $script:GetRobotHandGuiLineEnding)]
            $portHandle.ReadTimeout = 25
            $portHandle.WriteTimeout = 1000
            $portHandle.Open()
            $script:RobotHandGuiState.SerialPort = $portHandle
            if (-not $script:RobotHandGuiState.PollTimer) {
                $timer = New-Object System.Windows.Forms.Timer
                $timer.Interval = 50
                $timer.Add_Tick({
                    if (-not $script:RobotHandGuiState.SerialPort -or -not $script:RobotHandGuiState.SerialPort.IsOpen) { return }
                    try {
                        $incoming = $script:RobotHandGuiState.SerialPort.ReadExisting()
                        if (-not [string]::IsNullOrEmpty($incoming)) {
                            & $script:HandleRobotHandIncomingData $incoming
                        }
                    } catch {
                        & $script:AppendRobotHandGuiLog "SYS" ("Serial read stopped: {0}" -f $_.Exception.Message)
                        & $script:DisconnectRobotHandGuiPort
                    }
                })
                $script:RobotHandGuiState.PollTimer = $timer
            }
            $script:RobotHandGuiState.PollTimer.Start()
            $script:RobotHandGuiState.ConnectButton.Text = "Disconnect"
            & $script:AppendRobotHandGuiLog "SYS" ("Connected to {0} at {1} baud" -f $portName, $parsedBaud)
            & $script:SetRobotHandGuiStatus ("Connected to {0}" -f $portName)
            $script:RobotHandGuiState.InputBox.Focus()
        } catch {
            & $script:AppendRobotHandGuiLog "SYS" ("Failed to open {0}: {1}" -f $portName, $_.Exception.Message)
            & $script:SetRobotHandGuiStatus ("Failed to open {0}" -f $portName)
        }
    }

    function New-LabeledTextBox([string]$Label, [string]$DefaultValue, [int]$Left, [int]$Top, [int]$Width = 60) {
        $labelControl = New-Object System.Windows.Forms.Label
        $labelControl.Text = $Label
        $labelControl.Location = New-Object System.Drawing.Point -ArgumentList $Left, ($Top + 4)
        $labelControl.AutoSize = $true
        $textBox = New-Object System.Windows.Forms.TextBox
        $textBox.Location = New-Object System.Drawing.Point -ArgumentList ($Left + 76), $Top
        $textBox.Size = New-Object System.Drawing.Size -ArgumentList $Width, 24
        $textBox.Text = $DefaultValue
        return @{ Label = $labelControl; TextBox = $textBox }
    }

    function Add-BuilderGroup([string]$Title, [int]$Height) {
        $group = New-Object System.Windows.Forms.GroupBox
        $group.Text = $Title
        $group.Width = 320
        $group.Height = $Height
        $group.Margin = New-Object System.Windows.Forms.Padding(3, 3, 3, 10)
        $buildersFlow.Controls.Add($group)
        $script:RobotHandGuiState.BuilderGroups += $group
        return $group
    }

    function Add-MotorSliderRow([int]$MotorId, [int]$MinValue, [int]$MaxValue) {
        $rowPanel = New-Object System.Windows.Forms.Panel
        $rowPanel.Dock = "Top"
        $rowPanel.Height = 78
        $topMargin = if ($MotorId -eq 1) { 8 } else { 3 }
        $rowPanel.Margin = New-Object System.Windows.Forms.Padding(3, $topMargin, 3, 10)
        $sliderLayout.Controls.Add($rowPanel, 0, $MotorId)

        $motorLabel = New-Object System.Windows.Forms.Label
        $motorLabel.Text = "Motor $MotorId"
        $motorLabel.Location = New-Object System.Drawing.Point -ArgumentList 6, 10
        $motorLabel.AutoSize = $true
        $rowPanel.Controls.Add($motorLabel)

        $valueLabel = New-Object System.Windows.Forms.Label
        $valueLabel.Text = [string]$MinValue
        $valueLabel.Location = New-Object System.Drawing.Point -ArgumentList 88, 10
        $valueLabel.Size = New-Object System.Drawing.Size -ArgumentList 72, 20
        $rowPanel.Controls.Add($valueLabel)

        $minLabel = New-Object System.Windows.Forms.Label
        $minLabel.Text = [string]$MinValue
        $minLabel.Location = New-Object System.Drawing.Point -ArgumentList 6, 48
        $minLabel.AutoSize = $true
        $rowPanel.Controls.Add($minLabel)

        $maxLabel = New-Object System.Windows.Forms.Label
        $maxLabel.Text = [string]$MaxValue
        $maxLabel.Anchor = [System.Windows.Forms.AnchorStyles]::Top -bor [System.Windows.Forms.AnchorStyles]::Right
        $maxLabel.Location = New-Object System.Drawing.Point -ArgumentList 300, 48
        $maxLabel.AutoSize = $true
        $rowPanel.Controls.Add($maxLabel)

        $trackBar = New-Object System.Windows.Forms.TrackBar
        $trackBar.Minimum = $MinValue
        $trackBar.Maximum = $MaxValue
        $trackBar.Tag = [string]$MotorId
        $trackBar.TickFrequency = 250
        $trackBar.SmallChange = 50
        $trackBar.LargeChange = 100
        $trackBar.AutoSize = $false
        $trackBar.Height = 30
        $trackBar.Left = 40
        $trackBar.Top = 34
        $trackBar.Width = 250
        $trackBar.Anchor = [System.Windows.Forms.AnchorStyles]::Top -bor [System.Windows.Forms.AnchorStyles]::Left -bor [System.Windows.Forms.AnchorStyles]::Right
        $rowPanel.Controls.Add($trackBar)

        $rowPanel.Add_SizeChanged({
            $trackBar.Width = [Math]::Max(120, $rowPanel.ClientSize.Width - 100)
            $maxLabel.Left = [Math]::Max($trackBar.Right + 8, $rowPanel.ClientSize.Width - $maxLabel.PreferredWidth - 6)
        }.GetNewClosure())

        $trackBar.Add_ValueChanged({
            param($sender, $eventArgs)
            $motorKey = [string]$sender.Tag
            $sliderInfo = $script:RobotHandGuiState.SliderControls[$motorKey]
            if ($null -ne $sliderInfo) {
                $sliderInfo.ValueLabel.Text = [string]$sender.Value
            }
            if ($sender.AccessibleDescription -eq "initializing") {
                return
            }
            if ($script:RobotHandGuiState.SuppressSliderSync) {
                return
            }
            $serialPort = $script:RobotHandGuiState.SerialPort
            if ($null -eq $serialPort -or -not $serialPort.IsOpen) {
                return
            }
            $selectedEnding = [string]$script:RobotHandGuiState.EndingCombo.SelectedItem
            if ([string]::IsNullOrWhiteSpace($selectedEnding)) {
                $selectedEnding = [string]$script:RobotHandGuiState.EndingCombo.Text
            }
            $lineEndingMap = $script:RobotHandGuiState.LineEndingMap
            if ($null -eq $lineEndingMap) {
                $lineEndingMap = @{
                    "none" = ""
                    "lf" = "`n"
                    "cr" = "`r"
                    "crlf" = "`r`n"
                }
            }
            if (-not $lineEndingMap.ContainsKey($selectedEnding)) {
                $selectedEnding = "lf"
            }
            try {
                $targetValue = [int]$sender.Value
                $commandText = "p{0},{1},5" -f $motorKey, $targetValue
                $serialPort.Write($commandText + $lineEndingMap[$selectedEnding])
                & $script:AppendRobotHandGuiLog "TX" $commandText
            } catch {
                $script:RobotHandGuiState.StatusLabel.Text = "Slider send failed for motor $motorKey"
            }
        })

        $trackBar.Add_MouseDown({
            param($sender, $eventArgs)
            if ($eventArgs.Button -ne [System.Windows.Forms.MouseButtons]::Left) {
                return
            }
            $trackWidth = [double]([Math]::Max(1, $sender.ClientSize.Width - 1))
            $ratio = [Math]::Max(0.0, [Math]::Min(1.0, $eventArgs.X / $trackWidth))
            $targetValue = [int][Math]::Round($sender.Minimum + (($sender.Maximum - $sender.Minimum) * $ratio))
            $sender.Value = [Math]::Max($sender.Minimum, [Math]::Min($sender.Maximum, $targetValue))
        })

        $trackBar.Add_MouseWheel({
            param($sender, $eventArgs)
            $step = 50
            $relative = $sender.Value - $sender.Minimum
            $baseValue = $sender.Minimum + ([int][Math]::Floor($relative / $step) * $step)
            $targetValue = if ($eventArgs.Delta -gt 0) { $baseValue + $step } else { $baseValue }
            $sender.Value = [Math]::Max($sender.Minimum, [Math]::Min($sender.Maximum, $targetValue))
        })

        $script:RobotHandGuiState.SliderControls[[string]$MotorId] = @{
            TrackBar = $trackBar
            ValueLabel = $valueLabel
            MinLabel = $minLabel
            MaxLabel = $maxLabel
            Panel = $rowPanel
        }
    }

    function Add-QuickCommandGroup([string]$Title, [object[]]$Items) {
        $group = New-Object System.Windows.Forms.GroupBox
        $group.Text = $Title
        $group.Width = 320
        $group.Height = 114
        $group.Margin = New-Object System.Windows.Forms.Padding(3, 3, 3, 10)

        $groupLayout = New-Object System.Windows.Forms.FlowLayoutPanel
        $groupLayout.Dock = "Fill"
        $groupLayout.FlowDirection = "LeftToRight"
        $groupLayout.WrapContents = $true
        $groupLayout.AutoScroll = $false
        $groupLayout.Padding = New-Object System.Windows.Forms.Padding(8)
        $group.Controls.Add($groupLayout)

        foreach ($item in $Items) {
            $commandValue = $item.Command
            $button = New-Object System.Windows.Forms.Button
            $button.Text = $item.Label
            $button.Tag = $commandValue
            $button.Width = 96
            $button.Height = 34
            $button.Margin = New-Object System.Windows.Forms.Padding(6)
            if ($commandValue -eq "sa") {
                $button.BackColor = [System.Drawing.Color]::FromArgb(220, 64, 64)
                $button.ForeColor = [System.Drawing.Color]::Black
                $button.Font = New-Object System.Drawing.Font($button.Font, [System.Drawing.FontStyle]::Bold)
            }
            $button.Add_Click({
                param($sender, $eventArgs)
                & $script:SendRobotHandGuiCommand ([string]$sender.Tag)
            })
            $groupLayout.Controls.Add($button)
        }

        $quickLayout.Controls.Add($group)
        $script:RobotHandGuiState.QuickGroups += $group
        $script:RobotHandGuiState.QuickGroupLayouts[$group] = @{
            Layout = $groupLayout
            Count = $Items.Count
        }
    }

    $script:UpdateQuickCommandGroupLayout = {
        $buttonWidth = 96
        $buttonSpacing = 12
        $rowHeight = 46
        $groupHeaderHeight = 54
        $availableWidth = $quickLayout.ClientSize.Width - $quickLayout.Padding.Left - $quickLayout.Padding.Right
        if ($availableWidth -le 0) {
            return
        }

        foreach ($group in $script:RobotHandGuiState.QuickGroups) {
            $group.Width = [Math]::Max(220, $availableWidth - 6)

            $groupInfo = $script:RobotHandGuiState.QuickGroupLayouts[$group]
            $layout = $groupInfo.Layout
            $buttonCount = [int]$groupInfo.Count
            $innerWidth = $group.Width - $group.Padding.Left - $group.Padding.Right - $layout.Padding.Left - $layout.Padding.Right
            $buttonsPerRow = [Math]::Max(1, [int][Math]::Floor(($innerWidth + $buttonSpacing) / ($buttonWidth + $buttonSpacing)))
            $rowCount = [Math]::Max(1, [int][Math]::Ceiling($buttonCount / [double]$buttonsPerRow))
            $group.Height = $groupHeaderHeight + ($rowCount * $rowHeight)
        }

        $builderAvailableWidth = $buildersFlow.ClientSize.Width - $buildersFlow.Padding.Left - $buildersFlow.Padding.Right
        if ($builderAvailableWidth -gt 0) {
            foreach ($group in $script:RobotHandGuiState.BuilderGroups) {
                $group.Width = [Math]::Max(220, $builderAvailableWidth - 6)
            }
        }
    }

    Add-QuickCommandGroup "Initiate" @(
        @{ Label = "Help"; Command = "h" },
        @{ Label = "Calibrate Direction"; Command = "cd" },
        @{ Label = "Calibrate Position"; Command = "cp" }
    )
    Add-QuickCommandGroup "Movement" @(
        @{ Label = "Move Zero"; Command = "mz" },
        @{ Label = "Finger Demo"; Command = "fd" },
        @{ Label = "Peace"; Command = "fp" },
        @{ Label = "Rock On"; Command = "fr" },
        @{ Label = "Middle"; Command = "fm" }
    )
    Add-QuickCommandGroup "System" @(
        @{ Label = "Encoders"; Command = "e" },
        @{ Label = "Reset All"; Command = "ra" },
        @{ Label = "Stop All"; Command = "sa" }
    )

    foreach ($motorId in 1..8) {
        $sliderMax = if (($motorId % 2) -eq 0) { 3500 } else { 3000 }
        Add-MotorSliderRow $motorId 0 $sliderMax
    }

    $sliderUpdateButton.Add_Click({
        & $script:RequestRobotHandSliderSnapshot
    })

    $speedTrackBar.Add_ValueChanged({
        $speedValueLabel.Text = [string]$speedTrackBar.Value
    })

    $speedTrackBar.Add_Scroll({
        & $script:SendRobotHandGuiSpeedCommand $speedTrackBar.Value
    })

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
            & $script:AppendRobotHandGuiLog "SYS" "Jog command needs a motor and direction."
            return
        }
        $commandText = if ([string]::IsNullOrWhiteSpace($counts)) { "j{0}{1}" -f $motorId, $direction } else { "j{0}{1},{2},{3}" -f $motorId, $direction, $counts, ($(if ([string]::IsNullOrWhiteSpace($tolerance)) { "5" } else { $tolerance })) }
        & $script:SendRobotHandGuiCommand $commandText
    })
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
    $positionSendButton.Add_Click({ & $script:SendRobotHandGuiCommand ("p{0},{1},{2}" -f $positionMotorField.TextBox.Text.Trim(), $positionValueField.TextBox.Text.Trim(), $positionToleranceField.TextBox.Text.Trim()) })
    $positionGroup.Controls.Add($positionSendButton)

    $fingerGroup = Add-BuilderGroup "Finger Move" 272
    $fingerHeaderLabel = New-Object System.Windows.Forms.Label
    $fingerHeaderLabel.Text = "On       Finger     Proximal     Distal"
    $fingerHeaderLabel.Location = New-Object System.Drawing.Point -ArgumentList 12, 28
    $fingerHeaderLabel.AutoSize = $true
    $fingerGroup.Controls.Add($fingerHeaderLabel)

    $fingerRows = @()
    foreach ($fingerId in 1..4) {
        $rowTop = 54 + (($fingerId - 1) * 34)

        $fingerActiveCheck = New-Object System.Windows.Forms.CheckBox
        $fingerActiveCheck.Checked = $true
        $fingerActiveCheck.Location = New-Object System.Drawing.Point -ArgumentList 14, ($rowTop + 2)
        $fingerActiveCheck.Size = New-Object System.Drawing.Size -ArgumentList 18, 24
        $fingerGroup.Controls.Add($fingerActiveCheck)

        $fingerLabel = New-Object System.Windows.Forms.Label
        $fingerLabel.Text = [string]$fingerId
        $fingerLabel.Location = New-Object System.Drawing.Point -ArgumentList 56, ($rowTop + 4)
        $fingerLabel.AutoSize = $true
        $fingerGroup.Controls.Add($fingerLabel)

        $fingerProxBox = New-Object System.Windows.Forms.TextBox
        $fingerProxBox.Location = New-Object System.Drawing.Point -ArgumentList 106, $rowTop
        $fingerProxBox.Size = New-Object System.Drawing.Size -ArgumentList 72, 24
        $fingerProxBox.Text = "0"
        $fingerGroup.Controls.Add($fingerProxBox)

        $fingerDistBox = New-Object System.Windows.Forms.TextBox
        $fingerDistBox.Location = New-Object System.Drawing.Point -ArgumentList 196, $rowTop
        $fingerDistBox.Size = New-Object System.Drawing.Size -ArgumentList 72, 24
        $fingerDistBox.Text = "0"
        $fingerGroup.Controls.Add($fingerDistBox)

        $fingerRows += @{
            Id = $fingerId
            Active = $fingerActiveCheck
            Proximal = $fingerProxBox
            Distal = $fingerDistBox
        }
    }

    $fingerToleranceField = New-LabeledTextBox "Tolerance" "5" 12 196
    $fingerGroup.Controls.Add($fingerToleranceField.Label)
    $fingerGroup.Controls.Add($fingerToleranceField.TextBox)
    $fingerSendButton = New-Object System.Windows.Forms.Button
    $fingerSendButton.Text = "Send Fingers"
    $fingerSendButton.Location = New-Object System.Drawing.Point(196, 228)
    $fingerSendButton.Size = New-Object System.Drawing.Size(92, 30)
    $fingerSendButton.Add_Click({
        $tol = $fingerToleranceField.TextBox.Text.Trim()
        if ([string]::IsNullOrWhiteSpace($tol)) { $tol = "5" }
        $commands = @()
        foreach ($fingerRow in $fingerRows) {
            if (-not $fingerRow.Active.Checked) {
                continue
            }
            $commands += "f{0},{1},{2},{3}" -f $fingerRow.Id, $fingerRow.Proximal.Text.Trim(), $fingerRow.Distal.Text.Trim(), $tol
        }
        if ($commands.Count -eq 0) {
            & $script:AppendRobotHandGuiLog "SYS" "Select at least one active finger."
            return
        }
        & $script:SendRobotHandGuiCommand ($commands -join ";")
    })
    $fingerGroup.Controls.Add($fingerSendButton)

    $refreshButton.Add_Click({ & $script:RefreshRobotHandGuiPorts })
    $connectButton.Add_Click({
        if ($script:RobotHandGuiState.SerialPort -and $script:RobotHandGuiState.SerialPort.IsOpen) {
            & $script:DisconnectRobotHandGuiPort
        } else {
            & $script:ConnectRobotHandGuiPort
        }
    })
    $sendButton.Add_Click({ & $script:SendRobotHandGuiCommand })
    $clearRxButton.Add_Click({ $script:RobotHandGuiState.RxLogBox.Clear() })
    $clearTxButton.Add_Click({ $script:RobotHandGuiState.TxLogBox.Clear() })
    $quickLayout.Add_SizeChanged({ & $script:UpdateQuickCommandGroupLayout })
    $buildersFlow.Add_SizeChanged({ & $script:UpdateQuickCommandGroupLayout })
    $inputBox.Add_KeyDown({
        if ($_.KeyCode -eq [System.Windows.Forms.Keys]::Up) {
            $_.SuppressKeyPress = $true
            if (-not [string]::IsNullOrWhiteSpace($script:RobotHandGuiState.LastCommand)) {
                $script:RobotHandGuiState.InputBox.Text = $script:RobotHandGuiState.LastCommand
                $script:RobotHandGuiState.InputBox.SelectionStart = $script:RobotHandGuiState.InputBox.TextLength
            }
        } elseif ($_.KeyCode -eq [System.Windows.Forms.Keys]::Enter) {
            $_.SuppressKeyPress = $true
            & $script:SendRobotHandGuiCommand
        }
    })
    $endingCombo.Add_SelectedIndexChanged({
        if ($script:RobotHandGuiState.SerialPort -and $script:RobotHandGuiState.SerialPort.IsOpen) {
            $script:RobotHandGuiState.SerialPort.NewLine = $script:RobotHandGuiState.LineEndingMap[(& $script:GetRobotHandGuiLineEnding)]
        }
    })
    $form.Add_Shown({
        $script:RobotHandGuiState.ContentSplit.Panel1MinSize = 620
        $script:RobotHandGuiState.ContentSplit.Panel2MinSize = 300
        $availableWidth = $script:RobotHandGuiState.ContentSplit.ClientSize.Width
        if ($availableWidth -gt 980) {
            $leftWidth = [Math]::Max(620, $availableWidth - 370)
            $leftWidth = [Math]::Min($leftWidth, $availableWidth - 300)
            $script:RobotHandGuiState.ContentSplit.SplitterDistance = $leftWidth
        }
        $script:RobotHandGuiState.LogSplit.Panel1MinSize = 180
        $script:RobotHandGuiState.LogSplit.Panel2MinSize = 180
        $availableHeight = $script:RobotHandGuiState.LogSplit.ClientSize.Height
        if ($availableHeight -gt 360) {
            $topHeight = [Math]::Min(310, $availableHeight - 180)
            $topHeight = [Math]::Max($topHeight, 180)
            $script:RobotHandGuiState.LogSplit.SplitterDistance = $topHeight
        }
        & $script:UpdateQuickCommandGroupLayout
        & $script:RefreshRobotHandGuiPorts
        if ($Port) {
            & $script:ConnectRobotHandGuiPort
        }
        $script:RobotHandGuiState.InputBox.Focus()
    })
    $form.Add_FormClosing({ & $script:DisconnectRobotHandGuiPort })

    [void]$form.ShowDialog()
}
