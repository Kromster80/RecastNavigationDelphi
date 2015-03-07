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
  object Memo1: TMemo
    Left = 264
    Top = 560
    Width = 545
    Height = 177
    Anchors = [akLeft, akRight, akBottom]
    ScrollBars = ssVertical
    TabOrder = 1
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
    TabOrder = 3
  end
  object gbSample: TGroupBox
    Left = 816
    Top = 160
    Width = 265
    Height = 577
    Anchors = [akTop, akRight, akBottom]
    TabOrder = 2
  end
  object gbTool: TGroupBox
    Left = 8
    Top = 160
    Width = 249
    Height = 393
    TabOrder = 4
  end
  object rgInputMesh: TRadioGroup
    Left = 816
    Top = 72
    Width = 265
    Height = 49
    Anchors = [akTop, akRight]
    Caption = 'Input Mesh'
    Columns = 3
    ItemIndex = 1
    Items.Strings = (
      'Dungeon'
      'Nav_Test')
    TabOrder = 5
  end
  object rgChooseSample: TRadioGroup
    Left = 816
    Top = 32
    Width = 265
    Height = 34
    Anchors = [akTop, akRight]
    Caption = 'Choose Sample'
    Columns = 3
    ItemIndex = 1
    Items.Strings = (
      'Solo Mesh'
      'Tile Mesh'
      'Temp Obstacles')
    TabOrder = 6
    OnClick = rgChooseSampleClick
  end
  object CheckBox1: TCheckBox
    Left = 824
    Top = 8
    Width = 65
    Height = 17
    Anchors = [akTop, akRight]
    Caption = 'Show Log'
    Checked = True
    Enabled = False
    State = cbChecked
    TabOrder = 7
  end
  object btnBuild: TButton
    Left = 816
    Top = 128
    Width = 265
    Height = 25
    Anchors = [akTop, akRight]
    Caption = 'Build'
    TabOrder = 8
    OnClick = btnBuildClick
  end
  object Timer1: TTimer
    Interval = 25
    OnTimer = Timer1Timer
    Left = 392
    Top = 40
  end
end
