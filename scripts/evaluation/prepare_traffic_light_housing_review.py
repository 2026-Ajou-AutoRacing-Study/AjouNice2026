#!/usr/bin/env python3
"""Original-pixel housing review sheets, not annotations or model predictions."""
import argparse
import hashlib
import json
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--dataset', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--templates', type=Path)
    args = parser.parse_args()
    from PIL import Image, ImageDraw
    source = args.dataset.resolve(strict=True)
    rows = json.loads(source.read_text())['records']
    for row in rows:
        relative=Path(row['image_path'])
        if relative.is_absolute() or '..' in relative.parts:raise ValueError('Unsafe source JPEG path')
        image=(source.parent/relative).resolve(strict=True)
        if source.parent not in image.parents or hashlib.sha256(image.read_bytes()).hexdigest()!=row['image_sha256']:
            raise ValueError('Original JPEG SHA mismatch before review')
    args.output.mkdir(parents=True, exist_ok=False)
    nearest = getattr(Image, 'Resampling', Image).NEAREST
    entries = []
    if args.templates:
        templates = json.loads(args.templates.read_text())['groups']
        tiles = []
        for i, row in enumerate(rows):
            with Image.open(source.parent/row['image_path']) as original:
                original = original.convert('RGB')
                for native_id, bbox in templates[row['group_id']].items():
                    x1,y1,x2,y2 = bbox
                    box = [max(0,x1-3),max(0,y1-3),min(original.width,x2+3),min(original.height,y2+3)]
                    crop = original.crop(box)
                    tile = Image.new('RGB', (320,140), '#222222')
                    draw = ImageDraw.Draw(tile)
                    ratio = min(310/crop.width,100/crop.height)
                    zoom = crop.resize((round(crop.width*ratio),round(crop.height*ratio)),nearest)
                    tile.paste(zoom,((320-zoom.width)//2,32+(100-zoom.height)//2))
                    draw.text((3,2),f'row {i} {row["environment"]} {native_id}',fill='white')
                    draw.text((3,16),row['group_id'],fill='white')
                    entries.append({'record_index':i,'native_id':native_id,'xyxy':bbox,
                                    'source_image_sha256':row['image_sha256'],
                                    'sheet':f'crops_{len(tiles)//24:03d}.png',
                                    'tile':len(tiles)%24})
                    tiles.append(tile)
        for batch in range(0,len(tiles),24):
            page = Image.new('RGB',(1280,840),'#222222')
            for offset,tile in enumerate(tiles[batch:batch+24]):page.paste(tile,(offset%4*320,offset//4*140))
            page.save(args.output/f'crops_{batch//24:03d}.png')
    else:
        seen = set()
        for i,row in enumerate(rows):
            key = row['environment'],row['group_id']
            if key in seen:continue
            seen.add(key)
            with Image.open(source.parent/row['image_path']) as original:original = original.convert('RGB')
            page = Image.new('RGB',(1280,1010),'#222222');page.paste(original,(0,0))
            draw = ImageDraw.Draw(page)
            for j,box in enumerate(row['boxes']):
                x1,y1,x2,y2 = box['xyxy']
                draw.rectangle((x1,y1,x2,y2),outline='#ff8844')
                draw.text((x1,max(0,y1-14)),box['native_id'][-3:],fill='#ff8844')
                crop_box = [max(0,x1-10),max(0,y1-10),min(1280,x2+10),min(720,y2+10)]
                crop = original.crop(crop_box)
                ratio = min(250/crop.width,220/crop.height)
                zoom = crop.resize((round(crop.width*ratio),round(crop.height*ratio)),nearest)
                if j<5:
                    page.paste(zoom,(j*256,760))
                    draw.text((j*256,730),f'{box["native_id"][-3:]} proposal {crop_box}',fill='white')
            name = f'reference_{len(seen)-1:02d}.png';page.save(args.output/name)
            entries.append({'record_index':i,'environment':key[0],'group_id':key[1],
                            'source_image_sha256':row['image_sha256'],'sheet':name,'boxes':row['boxes']})
    with (args.output/'index.json').open('x') as stream:
        json.dump({'source':str(source),'source_sha256':hashlib.sha256(source.read_bytes()).hexdigest(),
                   'model_predictions_used':False,'review_does_not_approve_labels':True,'entries':entries},stream,indent=2)
    print(json.dumps({'entries':len(entries),'output':str(args.output)}))


if __name__=='__main__':main()
