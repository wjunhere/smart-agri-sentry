Component({
  properties: {
    text: { type: String, value: '' },
    type: { type: String, value: 'green' },
  },
  data: {
    _class: '',
  },
  observers: {
    'type': function(t: string) {
      const classMap: Record<string, string> = {
        green: 'badge badge-green',
        blue:  'badge badge-blue',
        red:   'badge badge-red',
        amber: 'badge badge-amber',
      };
      this.setData({ _class: classMap[t] || classMap.green });
    }
  }
})
