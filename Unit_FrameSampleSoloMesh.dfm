object FrameSampleSoloMesh: TFrameSampleSoloMesh
  Left = 0
  Top = 0
  Width = 265
  Height = 689
  Anchors = [akLeft, akTop, akBottom]
  TabOrder = 0
  DesignSize = (
    265
    689)
  object Label13: TLabel
    Left = 64
    Top = 304
    Width = 84
    Height = 13
    Caption = 'Max Sample Error'
  end
  object Label12: TLabel
    Left = 64
    Top = 280
    Width = 78
    Height = 13
    Caption = 'Sample Distance'
  end
  object Label11: TLabel
    Left = 64
    Top = 248
    Width = 67
    Height = 13
    Caption = 'Verts Per Poly'
  end
  object Label10: TLabel
    Left = 64
    Top = 224
    Width = 74
    Height = 13
    Caption = 'Max Edge Error'
  end
  object Label9: TLabel
    Left = 64
    Top = 200
    Width = 83
    Height = 13
    Caption = 'Max Edge Length'
  end
  object Label8: TLabel
    Left = 64
    Top = 136
    Width = 94
    Height = 13
    Caption = 'Merged Region Size'
  end
  object Label7: TLabel
    Left = 64
    Top = 112
    Width = 74
    Height = 13
    Caption = 'Min Region Size'
  end
  object Label6: TLabel
    Left = 192
    Top = 80
    Width = 49
    Height = 13
    Caption = 'Max Slope'
  end
  object Label5: TLabel
    Left = 192
    Top = 56
    Width = 48
    Height = 13
    Caption = 'Max Climb'
  end
  object Label4: TLabel
    Left = 192
    Top = 32
    Width = 64
    Height = 13
    Caption = 'Agent Radius'
  end
  object Label2: TLabel
    Left = 63
    Top = 32
    Width = 51
    Height = 13
    Caption = 'Cell Height'
  end
  object Label1: TLabel
    Left = 63
    Top = 7
    Width = 39
    Height = 13
    Caption = 'Cell Size'
  end
  object Label3: TLabel
    Left = 192
    Top = 8
    Width = 63
    Height = 13
    Caption = 'Agent Height'
  end
  object seCellSize: TSpinEdit
    Left = 8
    Top = 8
    Width = 49
    Height = 22
    MaxValue = 10
    MinValue = 1
    TabOrder = 0
    Value = 3
  end
  object seMaxSampleError: TSpinEdit
    Left = 8
    Top = 304
    Width = 49
    Height = 22
    MaxValue = 16
    MinValue = 0
    TabOrder = 1
    Value = 1
  end
  object seSampleDistance: TSpinEdit
    Left = 8
    Top = 280
    Width = 49
    Height = 22
    MaxValue = 16
    MinValue = 0
    TabOrder = 2
    Value = 6
  end
  object seVertsPerPoly: TSpinEdit
    Left = 8
    Top = 248
    Width = 49
    Height = 22
    MaxValue = 12
    MinValue = 3
    TabOrder = 3
    Value = 6
  end
  object seMaxEdgeError: TSpinEdit
    Left = 8
    Top = 224
    Width = 49
    Height = 22
    MaxValue = 30
    MinValue = 1
    TabOrder = 4
    Value = 13
  end
  object seMaxEdgeLength: TSpinEdit
    Left = 8
    Top = 200
    Width = 49
    Height = 22
    MaxValue = 50
    MinValue = 0
    TabOrder = 5
    Value = 12
  end
  object rgPartitioning: TRadioGroup
    Left = 8
    Top = 160
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
    Top = 136
    Width = 49
    Height = 22
    MaxValue = 150
    MinValue = 0
    TabOrder = 7
    Value = 20
  end
  object seMinRegionSize: TSpinEdit
    Left = 8
    Top = 112
    Width = 49
    Height = 22
    MaxValue = 150
    MinValue = 0
    TabOrder = 8
    Value = 8
  end
  object seMaxSlope: TSpinEdit
    Left = 136
    Top = 80
    Width = 49
    Height = 22
    MaxValue = 90
    MinValue = 0
    TabOrder = 9
    Value = 45
  end
  object seMaxClimb: TSpinEdit
    Left = 136
    Top = 56
    Width = 49
    Height = 22
    MaxValue = 50
    MinValue = 1
    TabOrder = 10
    Value = 9
  end
  object seAgentRadius: TSpinEdit
    Left = 136
    Top = 32
    Width = 49
    Height = 22
    MaxValue = 50
    MinValue = 0
    TabOrder = 11
    Value = 6
  end
  object seAgentHeight: TSpinEdit
    Left = 136
    Top = 8
    Width = 49
    Height = 22
    MaxValue = 50
    MinValue = 1
    TabOrder = 12
    Value = 20
  end
  object seCellHeight: TSpinEdit
    Left = 8
    Top = 32
    Width = 49
    Height = 22
    MaxValue = 10
    MinValue = 1
    TabOrder = 13
    Value = 2
  end
  object chkKeepIntermediateResults: TCheckBox
    Left = 8
    Top = 336
    Width = 97
    Height = 17
    Caption = 'Keep Itermediate Results'
    TabOrder = 14
  end
  object rgDrawMode: TRadioGroup
    Left = 8
    Top = 360
    Width = 249
    Height = 321
    Anchors = [akLeft, akTop, akBottom]
    Caption = ' Draw Mode '
    Items.Strings = (
      '1'
      '2'
      '3')
    TabOrder = 15
  end
end
