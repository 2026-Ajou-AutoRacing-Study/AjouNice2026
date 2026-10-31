#!/usr/bin/env python3
"""Source-pixel contact sheet only; no model/projection or automatic identity GT."""
import argparse
import hashlib
import json
from pathlib import Path


def build(root, output, window):
    from PIL import Image, ImageDraw
    root, output = Path(root).resolve(), Path(output).resolve()
    allowed = Path(__file__).resolve().parents[2] / 'artifacts/evaluation'
    output.relative_to(allowed)
    if output.exists():
        raise FileExistsError('Use a fresh review directory')
    rows, cards = [], []
    for cell in ('sunny13','sunny11','sunny15','foggy13','foggy11','foggy15'):
        manifest = root / cell / 'manifest.json'
        document = json.loads(manifest.read_text())
        for frame in document['frames']:
            if not frame.get('accepted_for_sample_quota'):
                continue
            path = manifest.parent / frame['image_path']
            digest = hashlib.sha256(path.read_bytes()).hexdigest()
            if digest != frame['sha256']:
                raise ValueError('Original JPEG changed')
            with Image.open(path) as image:
                image = image.convert('RGB')
                crop = image.crop(window)
                enlarged = crop.resize((crop.width*2,crop.height*2), Image.Resampling.NEAREST)
            card = Image.new('RGB',(enlarged.width,enlarged.height+42),'white')
            card.paste(enlarged,(0,42))
            ImageDraw.Draw(card).text((5,4),cell+' native='+str(frame['requested_state'])+'\noriginal pixels, x/y='+str(window[:2]),fill='black')
            cards.append(card)
            rows.append(dict(cell=cell, source=str(path), sha256=digest,
                             native_id=frame['requested_native_id'], native_state=frame['requested_state'],
                             native_verified=frame['native_GT_verified'], crop_window=list(window),
                             source_stamp=frame['header_stamp'], identity_review='pending'))
    if not cards:
        raise ValueError('No accepted original frames')
    cols=5;cw,ch=cards[0].size
    sheet=Image.new('RGB',(cols*cw,((len(cards)+cols-1)//cols)*ch),(215,215,215))
    for index,card in enumerate(cards):
        sheet.paste(card,((index%cols)*cw,(index//cols)*ch))
    output.mkdir(parents=True)
    sheet.save(output/'contact.png')
    (output/'index.json').write_text(json.dumps(dict(scope='manual pixel review pending; not auto labels',records=rows),indent=2)+'\n')
    print(json.dumps(dict(output=str(output),images=len(rows))))


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--window',type=int,nargs=4,required=True)
    args=parser.parse_args()
    if not (0<=args.window[0]<args.window[2]<=1280 and 0<=args.window[1]<args.window[3]<=720):
        parser.error('Explicit source-pixel crop must fit the 1280x720 image')
    build(args.root,args.output,args.window)


if __name__=='__main__':main()
