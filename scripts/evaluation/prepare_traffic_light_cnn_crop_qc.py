#!/usr/bin/env python3
"""Lossless contact sheets for explicit visual review; never approves a label."""
import argparse
import hashlib
import json
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--dataset', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    from PIL import Image, ImageDraw
    source = args.dataset.resolve(strict=True)
    raw = source.read_bytes()
    document = json.loads(raw)
    rows = [(i, row) for i, row in enumerate(document['records'])
            if row.get('provenance', {}).get('yolo_crop_proposal')]
    args.output.mkdir(parents=True, exist_ok=False)
    index = []
    resampling = getattr(Image, 'Resampling', Image)
    for batch in range(0, len(rows), 18):
        sheet = Image.new('RGB', (1200, 1080), '#202020')
        draw = ImageDraw.Draw(sheet)
        for offset, (record_index, row) in enumerate(rows[batch:batch+18]):
            original = Image.open(source.parent/row['image_path']).convert('RGB')
            box = row['crop_integer_xyxy']
            crop = original.crop(box)
            pixel_sha = hashlib.sha256(crop.tobytes()).hexdigest()
            proposal = row['provenance']['yolo_crop_proposal']
            if pixel_sha != proposal['crop_pixel_sha256_rgb']:
                raise ValueError('Proposal pixels changed')
            x, y = offset % 3*400, offset//3*180
            ratio = min(390/crop.width, 112/crop.height)
            fitted = crop.resize((max(1, round(crop.width*ratio)), max(1, round(crop.height*ratio))),
                                 resampling.NEAREST)
            sheet.paste(fitted, (x+(400-fitted.width)//2, y+46+(112-fitted.height)//2))
            bits = ''.join(str(row['labels'][key]) for key in ('red','yellow','green','left_arrow'))
            draw.text((x+5,y+4), f'row {record_index} {row["environment"]} {row["split"]} RYGL={bits}', fill='white')
            draw.text((x+5,y+20), f'{row["group_id"]} {proposal["native_id_joined"]}', fill='white')
            draw.text((x+5,y+160), f'{crop.width}x{crop.height} proposal; NOT approved', fill='#ffcc88')
            index.append({'record_index':record_index,'sheet':f'page_{batch//18:02d}.png',
                          'tile_xyxy':[x,y,x+400,y+180],'image_path':row['image_path'],
                          'crop_xyxy':box,'pixel_sha256_rgb':pixel_sha})
        sheet.save(args.output/f'page_{batch//18:02d}.png')
    with (args.output/'index.json').open('x') as stream:
        json.dump({'source':str(source),'source_sha256':hashlib.sha256(raw).hexdigest(),
                   'no_QC_flags_changed':True,'proposal_rows':index},stream,indent=2)
    print(json.dumps({'proposals':len(rows),'pages':(len(rows)+17)//18,'output':str(args.output)}))


if __name__ == '__main__':
    main()
