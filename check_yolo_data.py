import json, os

base = '/mnt/d/wjun/data/yolo'
for split in ['train', 'valid', 'test']:
    path = os.path.join(base, split, '_annotations.coco.json')
    with open(path) as f:
        d = json.load(f)
    cats = {c['id']: c['name'] for c in d['categories']}
    counts = {}
    for a in d['annotations']:
        name = cats[a['category_id']]
        counts[name] = counts.get(name, 0) + 1
    print(f'{split}: {len(d["images"])} images, {len(d["annotations"])} annotations')
    for k, v in sorted(counts.items()):
        print(f'  {k}: {v}')
