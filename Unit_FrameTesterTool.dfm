object FrameTesterTool: TFrameTesterTool
  Left = 0
  Top = 0
  Width = 320
  Height = 386
  TabOrder = 0
  object Label1: TLabel
    Left = 152
    Top = 160
    Width = 63
    Height = 13
    Caption = 'Include Flags'
  end
  object Label2: TLabel
    Left = 152
    Top = 248
    Width = 65
    Height = 13
    Caption = 'Exclude Flags'
  end
  object rgToolMode: TRadioGroup
    Left = 8
    Top = 8
    Width = 305
    Height = 137
    Caption = 'rgToolMode'
    Items.Strings = (
      'TOOLMODE_PATHFIND_FOLLOW'
      'TOOLMODE_PATHFIND_STRAIGHT'
      'TOOLMODE_PATHFIND_SLICED'
      'TOOLMODE_RAYCAST'
      'TOOLMODE_DISTANCE_TO_WALL'
      'TOOLMODE_FIND_POLYS_IN_CIRCLE'
      'TOOLMODE_FIND_POLYS_IN_SHAPE'
      'TOOLMODE_FIND_LOCAL_NEIGHBOURHOOD')
    TabOrder = 0
  end
  object rgVerticesAtCrossings: TRadioGroup
    Left = 8
    Top = 152
    Width = 129
    Height = 65
    Caption = ' Vertices at crossings '
    Items.Strings = (
      'None'
      'Area'
      'All')
    TabOrder = 1
  end
  object btnSetRandomStart: TButton
    Left = 8
    Top = 224
    Width = 129
    Height = 25
    Caption = 'Set Random Start'
    TabOrder = 2
  end
  object btnSetRandomEnd: TButton
    Left = 8
    Top = 256
    Width = 129
    Height = 25
    Caption = 'Set Random End'
    TabOrder = 3
  end
  object btnMakeRandomPoints: TButton
    Left = 8
    Top = 288
    Width = 129
    Height = 25
    Caption = 'Make Random Points'
    TabOrder = 4
  end
  object btnMakeRandomPointsAround: TButton
    Left = 8
    Top = 320
    Width = 129
    Height = 25
    Caption = 'Make Random Points Around'
    TabOrder = 5
  end
  object cbInclWalk: TCheckBox
    Left = 152
    Top = 176
    Width = 49
    Height = 17
    Caption = 'Walk'
    TabOrder = 6
  end
  object cbInclSwim: TCheckBox
    Left = 152
    Top = 192
    Width = 49
    Height = 17
    Caption = 'Swim'
    TabOrder = 7
  end
  object cbInclDoor: TCheckBox
    Left = 152
    Top = 208
    Width = 49
    Height = 17
    Caption = 'Door'
    TabOrder = 8
  end
  object cbInclJump: TCheckBox
    Left = 152
    Top = 224
    Width = 49
    Height = 17
    Caption = 'Jump'
    TabOrder = 9
  end
  object cbExclWalk: TCheckBox
    Left = 152
    Top = 264
    Width = 49
    Height = 17
    Caption = 'Walk'
    TabOrder = 10
  end
  object cbExclSwim: TCheckBox
    Left = 152
    Top = 280
    Width = 49
    Height = 17
    Caption = 'Swim'
    TabOrder = 11
  end
  object cbExclDoor: TCheckBox
    Left = 152
    Top = 296
    Width = 49
    Height = 17
    Caption = 'Door'
    TabOrder = 12
  end
  object cbExclJump: TCheckBox
    Left = 152
    Top = 312
    Width = 49
    Height = 17
    Caption = 'Jump'
    TabOrder = 13
  end
end
