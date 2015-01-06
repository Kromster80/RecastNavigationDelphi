object Form1: TForm1
  Left = 58
  Top = 8
  ClientHeight = 745
  ClientWidth = 1089
  Color = clBtnFace
  Constraints.MinHeight = 360
  Constraints.MinWidth = 800
  Font.Charset = DEFAULT_CHARSET
  Font.Color = clWindowText
  Font.Height = -11
  Font.Name = 'MS Sans Serif'
  Font.Style = []
  OldCreateOrder = False
  Position = poScreenCenter
  OnCreate = FormCreate
  OnDestroy = FormDestroy
  OnMouseWheel = FormMouseWheel
  OnResize = FormResize
  DesignSize = (
    1089
    745)
  PixelsPerInch = 96
  TextHeight = 13
  object Panel3: TPanel
    Left = 264
    Top = 8
    Width = 545
    Height = 545
    Anchors = [akLeft, akTop, akRight, akBottom]
    BevelOuter = bvNone
    Color = clSilver
    FullRepaint = False
    ParentBackground = False
    TabOrder = 0
    OnMouseDown = Panel3MouseDown
    OnMouseMove = Panel3MouseMove
    object Panel1: TPanel
      Left = 0
      Top = 0
      Width = 185
      Height = 25
      Caption = 'Dummy Panel for AA initialization'
      TabOrder = 0
      Visible = False
    end
  end
  object Button1: TButton
    Left = 816
    Top = 496
    Width = 265
    Height = 25
    Anchors = [akTop, akRight]
    Caption = 'Build'
    TabOrder = 1
    OnClick = Button1Click
  end
  object rgDrawMode: TRadioGroup
    Left = 816
    Top = 528
    Width = 265
    Height = 273
    Anchors = [akTop, akRight]
    Caption = ' Draw Mode '
    Items.Strings = (
      '1'
      '2'
      '3')
    TabOrder = 2
    OnClick = rgDrawModeClick
  end
  object Memo1: TMemo
    Left = 264
    Top = 560
    Width = 545
    Height = 177
    Anchors = [akLeft, akRight, akBottom]
    ScrollBars = ssVertical
    TabOrder = 3
  end
  object rgTool: TRadioGroup
    Left = 8
    Top = 8
    Width = 249
    Height = 145
    Caption = ' Tool '
    Items.Strings = (
      '1'
      '2'
      '3')
    TabOrder = 5
    OnClick = rgToolClick
  end
  object GroupBox1: TGroupBox
    Left = 816
    Top = 8
    Width = 265
    Height = 481
    Anchors = [akTop, akRight]
    TabOrder = 4
    object Label13: TLabel
      Left = 64
      Top = 408
      Width = 83
      Height = 13
      Caption = 'Max Sample Error'
    end
    object Label12: TLabel
      Left = 64
      Top = 384
      Width = 80
      Height = 13
      Caption = 'Sample Distance'
    end
    object Label11: TLabel
      Left = 64
      Top = 352
      Width = 66
      Height = 13
      Caption = 'Verts Per Poly'
    end
    object Label10: TLabel
      Left = 64
      Top = 328
      Width = 73
      Height = 13
      Caption = 'Max Edge Error'
    end
    object Label9: TLabel
      Left = 64
      Top = 304
      Width = 84
      Height = 13
      Caption = 'Max Edge Length'
    end
    object Label8: TLabel
      Left = 64
      Top = 240
      Width = 96
      Height = 13
      Caption = 'Merged Region Size'
    end
    object Label7: TLabel
      Left = 64
      Top = 216
      Width = 77
      Height = 13
      Caption = 'Min Region Size'
    end
    object Label6: TLabel
      Left = 192
      Top = 184
      Width = 50
      Height = 13
      Caption = 'Max Slope'
    end
    object Label5: TLabel
      Left = 192
      Top = 160
      Width = 48
      Height = 13
      Caption = 'Max Climb'
    end
    object Label4: TLabel
      Left = 192
      Top = 136
      Width = 64
      Height = 13
      Caption = 'Agent Radius'
    end
    object Label3: TLabel
      Left = 192
      Top = 112
      Width = 62
      Height = 13
      Caption = 'Agent Height'
    end
    object Label2: TLabel
      Left = 63
      Top = 136
      Width = 51
      Height = 13
      Caption = 'Cell Height'
    end
    object Label1: TLabel
      Left = 63
      Top = 111
      Width = 40
      Height = 13
      Caption = 'Cell Size'
    end
    object seCellSize: TSpinEdit
      Left = 8
      Top = 112
      Width = 49
      Height = 22
      MaxValue = 10
      MinValue = 1
      TabOrder = 0
      Value = 3
    end
    object seMaxSampleError: TSpinEdit
      Left = 8
      Top = 408
      Width = 49
      Height = 22
      MaxValue = 16
      MinValue = 0
      TabOrder = 1
      Value = 1
    end
    object seSampleDistance: TSpinEdit
      Left = 8
      Top = 384
      Width = 49
      Height = 22
      MaxValue = 16
      MinValue = 0
      TabOrder = 2
      Value = 6
    end
    object seVertsPerPoly: TSpinEdit
      Left = 8
      Top = 352
      Width = 49
      Height = 22
      MaxValue = 12
      MinValue = 3
      TabOrder = 3
      Value = 6
    end
    object seMaxEdgeError: TSpinEdit
      Left = 8
      Top = 328
      Width = 49
      Height = 22
      MaxValue = 30
      MinValue = 1
      TabOrder = 4
      Value = 13
    end
    object seMaxEdgeLength: TSpinEdit
      Left = 8
      Top = 304
      Width = 49
      Height = 22
      MaxValue = 50
      MinValue = 0
      TabOrder = 5
      Value = 12
    end
    object rgPartitioning: TRadioGroup
      Left = 8
      Top = 264
      Width = 249
      Height = 33
      Caption = ' Partitioning '
      Columns = 3
      Enabled = False
      ItemIndex = 0
      Items.Strings = (
        'Watershed'
        'Monotone'
        'Layers')
      TabOrder = 6
    end
    object seMergedRegionSize: TSpinEdit
      Left = 8
      Top = 240
      Width = 49
      Height = 22
      MaxValue = 150
      MinValue = 0
      TabOrder = 7
      Value = 20
    end
    object seMinRegionSize: TSpinEdit
      Left = 8
      Top = 216
      Width = 49
      Height = 22
      MaxValue = 150
      MinValue = 0
      TabOrder = 8
      Value = 8
    end
    object seMaxSlope: TSpinEdit
      Left = 136
      Top = 184
      Width = 49
      Height = 22
      MaxValue = 90
      MinValue = 0
      TabOrder = 9
      Value = 45
    end
    object seMaxClimb: TSpinEdit
      Left = 136
      Top = 160
      Width = 49
      Height = 22
      MaxValue = 50
      MinValue = 1
      TabOrder = 10
      Value = 9
    end
    object seAgentRadius: TSpinEdit
      Left = 136
      Top = 136
      Width = 49
      Height = 22
      MaxValue = 50
      MinValue = 0
      TabOrder = 11
      Value = 6
    end
    object seAgentHeight: TSpinEdit
      Left = 136
      Top = 112
      Width = 49
      Height = 22
      MaxValue = 50
      MinValue = 1
      TabOrder = 12
      Value = 20
    end
    object seCellHeight: TSpinEdit
      Left = 8
      Top = 136
      Width = 49
      Height = 22
      MaxValue = 10
      MinValue = 1
      TabOrder = 13
      Value = 2
    end
    object chkKeepIntermediateResults: TCheckBox
      Left = 8
      Top = 448
      Width = 97
      Height = 17
      Caption = 'Keep Itermediate Results'
      TabOrder = 14
      OnClick = chkKeepIntermediateResultsClick
    end
    object rgInputMesh: TRadioGroup
      Left = 8
      Top = 72
      Width = 249
      Height = 33
      Caption = 'Input Mesh'
      Columns = 2
      ItemIndex = 1
      Items.Strings = (
        'Dungeon'
        'Nav_Test')
      TabOrder = 15
    end
    object rgChooseSample: TRadioGroup
      Left = 8
      Top = 32
      Width = 249
      Height = 34
      Caption = 'Choose Sample'
      Columns = 3
      Enabled = False
      ItemIndex = 0
      Items.Strings = (
        'Solo Mesh'
        'Tile Mesh'
        'Temp Obstacles')
      TabOrder = 16
    end
    object CheckBox2: TCheckBox
      Left = 136
      Top = 9
      Width = 81
      Height = 17
      Caption = 'Show Tools'
      Checked = True
      Enabled = False
      State = cbChecked
      TabOrder = 17
    end
    object CheckBox1: TCheckBox
      Left = 8
      Top = 8
      Width = 65
      Height = 17
      Caption = 'Show Log'
      Checked = True
      Enabled = False
      State = cbChecked
      TabOrder = 18
    end
  end
  object gbTool: TGroupBox
    Left = 8
    Top = 160
    Width = 249
    Height = 393
    TabOrder = 6
  end
  object Timer1: TTimer
    Interval = 25
    OnTimer = Timer1Timer
    Left = 392
    Top = 40
  end
end
